"""Host-side link to the waveform_daq firmware, plus a software emulator of it.

Wire format (MCU -> PC): ASCII reply lines ("OK ..." / "ERR ...") interleaved with sample
bytes. A sample is two bytes: byte0 = 0x80 | (v >> 5), byte1 = v & 0x1F (v = 10-bit ADC
count). Bit 7 marks the first byte so the stream self-synchronises after any lost byte.
"""
import re
import time

import numpy as np

DDS_RATE_HZ = 16e6 / 256 / 8          # 7812.5 Hz, matches the firmware
ADC_TICK_HZ = 250000
ADC_DIV_MIN, ADC_DIV_MAX = 50, 256
FILTER_R, FILTER_C = 2200.0, 47e-9     # two-stage RC low-pass on the PWM output (docs/circuit.png)


def ladder_gain(f, r=FILTER_R, c=FILTER_C):
    """|H(f)| of the two-stage RC ladder, H = 1 / (1 + 3x + x^2), x = j*2*pi*f*R*C."""
    x = 1j * 2 * np.pi * f * r * c
    return float(abs(1 / (1 + 3 * x + x * x)))


class StreamDecoder:
    """Incremental decoder for the mixed ASCII + binary sample stream."""

    def __init__(self):
        self._hi = None
        self._text = bytearray()

    def feed(self, data):
        samples, lines = [], []
        for b in data:
            if b & 0x80:
                self._hi = b & 0x7F
            elif self._hi is not None:
                samples.append((self._hi << 5) | (b & 0x1F))
                self._hi = None
            elif b == 0x0A:
                line = self._text.decode("ascii", "replace").strip()
                self._text.clear()
                if line:
                    lines.append(line)
            elif b != 0x0D:
                self._text.append(b)
        return samples, lines


class DAQ:
    """Command/stream interface. `transport` needs write(), read(n), in_waiting."""

    def __init__(self, transport):
        self.t = transport
        self.dec = StreamDecoder()
        self._samples = []
        self.fs = None

    @classmethod
    def open_serial(cls, port, baud=115200):
        import serial
        ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(2.0)                    # Uno resets when the port opens
        ser.reset_input_buffer()
        return cls(ser)

    def _pump(self):
        n = getattr(self.t, "in_waiting", 0) or 1
        data = self.t.read(n)
        if not data:
            return []
        s, lines = self.dec.feed(data)
        self._samples.extend(s)
        return lines

    def command(self, text, timeout=2.0):
        self.t.write((text + "\n").encode("ascii"))
        deadline = time.time() + timeout
        while time.time() < deadline:
            for line in self._pump():
                if line.startswith("OK") or line.startswith("ERR"):
                    if line.startswith("ERR"):
                        raise RuntimeError("device error for %r: %s" % (text, line))
                    return line
        raise TimeoutError("no reply to %r" % text)

    @staticmethod
    def _field(reply, key):
        m = re.search(key + r"=([-+0-9.eE]+)", reply)
        return float(m.group(1)) if m else None

    def set_rate(self, hz):
        self.fs = self._field(self.command("RATE %g" % hz), "fs")
        return self.fs

    def generate(self, shape, freq, amp=None, duty=None):
        """shape: 'sin', 'tri' or 'sq'. Returns the actual frequency the device programmed."""
        if shape == "sq":
            cmd = "SQ %g" % freq + ("" if duty is None else " %g" % duty)
        else:
            cmd = "%s %g" % (shape.upper(), freq) + ("" if amp is None else " %g" % amp)
        return self._field(self.command(cmd), "f")

    def off(self):
        self.command("OFF")

    def capture(self, n, timeout=None, discard=4):
        """Capture n samples at the current rate. Returns raw ADC counts."""
        if self.fs is None:
            self.set_rate(2000)
        self._samples = []
        self.command("START")
        need = n + discard
        timeout = timeout or (need / self.fs * 2 + 3)
        deadline = time.time() + timeout
        while len(self._samples) < need and time.time() < deadline:
            self._pump()
        self.command("STOP")
        out = np.array(self._samples[discard:need], dtype=int)
        if len(out) < n:
            raise TimeoutError("only %d of %d samples received" % (len(out), n))
        return out

    def stream_chunks(self):
        """Generator-style helper for live display: returns samples received since last call."""
        self._pump()
        s, self._samples = self._samples, []
        return s


# ---------------------------------------------------------------------------------------
# Emulator of the firmware (same command set and the same frequency quantisation).
# ---------------------------------------------------------------------------------------
class FakeDevice:
    """Software stand-in for the Arduino, for testing without hardware.

    The ADC input is wired (virtually) to the generator output, like the loopback test
    (source="auto": filter output if a sine/triangle is active, else the square-wave pin).
    Sine amplitude includes the RC filter droop; triangle is ideal (the filter would also
    round its peaks, which this emulator does not model):
    v = vcc * (0.5 + 0.5 * amp * shape(phase)) + gaussian noise, quantised to 10 bits.
    """

    def __init__(self, vcc=5.0, noise_v=0.002, chunk=64, seed=0, source="auto"):
        self.vcc, self.noise_v, self.chunk, self.source = vcc, noise_v, chunk, source
        self.rng = np.random.default_rng(seed)
        self.rx = bytearray()
        self.tx = bytearray()
        self.streaming = False
        self.adc_div = 125
        self.n = 0
        self.shape, self.f_dds, self.amp = None, 0.0, 1.0
        self.f_sq, self.duty = 0.0, 50.0

    @property
    def fs(self):
        return ADC_TICK_HZ / self.adc_div

    @property
    def in_waiting(self):
        self._produce()
        return len(self.tx)

    def write(self, data):
        self.rx.extend(data)
        while b"\n" in self.rx:
            i = self.rx.index(b"\n")
            line = bytes(self.rx[:i]).decode().strip()
            del self.rx[:i + 1]
            if line:
                self._handle(line)

    def read(self, n=1):
        self._produce()
        out = bytes(self.tx[:n])
        del self.tx[:n]
        return out

    def _reply(self, s):
        self.tx.extend((s + "\n").encode())

    def _handle(self, line):
        p = line.split()
        cmd, args = p[0].upper(), [float(x) for x in p[1:]]
        if cmd == "RATE":
            d = int(round(ADC_TICK_HZ / args[0]))
            self.adc_div = min(max(d, ADC_DIV_MIN), ADC_DIV_MAX)
            self._reply("OK RATE fs=%.3f" % self.fs)
        elif cmd == "START":
            self.streaming, self.n = True, 0
            self._reply("OK START fs=%.3f" % self.fs)
        elif cmd == "STOP":
            self.streaming = False
            self._reply("OK STOP drops=0")
        elif cmd in ("SIN", "TRI"):
            step = int(args[0] * (2 ** 32 / DDS_RATE_HZ) + 0.5)
            self.f_dds = step / (2 ** 32 / DDS_RATE_HZ)
            self.shape = cmd.lower()
            self.amp = (args[1] if len(args) > 1 else 100.0) / 100.0
            self._reply("OK %s f=%.4f" % (cmd, self.f_dds))
        elif cmd == "SQ":
            self.duty = args[1] if len(args) > 1 else 50.0
            for pre in (1, 8, 64, 256, 1024):
                top = 16e6 / (pre * args[0]) - 1
                if 1 <= top <= 65535:
                    icr = int(top + 0.5)
                    self.f_sq = 16e6 / (pre * (icr + 1))
                    break
            self._reply("OK SQ f=%.3f duty=%.2f" % (self.f_sq, self.duty))
        elif cmd == "OFF":
            self.shape, self.f_sq = None, 0.0
            self._reply("OK OFF")
        else:
            self._reply("ERR unknown")

    def _signal(self, t):
        use_sq = self.source == "sq" or (self.source == "auto" and self.shape is None)
        if use_sq:
            if self.f_sq == 0:
                return np.zeros_like(t)
            ph = (t * self.f_sq) % 1.0
            return self.vcc * (ph < self.duty / 100.0).astype(float)
        if self.shape is None:
            return np.zeros_like(t)
        ph = (t * self.f_dds) % 1.0
        if self.shape == "sin":
            w = ladder_gain(self.f_dds) * np.sin(2 * np.pi * ph)   # filter passband droop
        else:
            w = 4 * np.abs(ph - 0.5) - 1          # triangle -1..1
            w = -w
        return self.vcc * (0.5 + 0.5 * self.amp * w)

    def _produce(self):
        if not self.streaming:
            return
        idx = np.arange(self.n, self.n + self.chunk)
        v = self._signal(idx / self.fs) + self.rng.normal(0, self.noise_v, len(idx))
        counts = np.clip(np.round(v / self.vcc * 1024), 0, 1023).astype(int)
        for c in counts:
            self.tx.append(0x80 | (c >> 5))
            self.tx.append(c & 0x1F)
        self.n += self.chunk
