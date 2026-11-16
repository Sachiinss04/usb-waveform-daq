#!/usr/bin/env python3
"""USB waveform generator / data acquisition host application.

  python daq_app.py --port COM3 gen sin 50 --amp 80
  python daq_app.py --port COM3 capture --seconds 2 --png capture.png --csv capture.csv
  python daq_app.py --port COM3 live
  python daq_app.py --port COM3 sweep --out ../results/hardware_measurements.csv
  python daq_app.py report ../results/hardware_measurements.csv

Add --sim to run against the built-in firmware emulator (no hardware needed).
"""
import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analysis import analyze, counts_to_volts   # noqa: E402
from daqlink import DAQ, FakeDevice, ladder_gain  # noqa: E402

SWEEP_POINTS = [("sin", f) for f in (5, 10, 20, 50, 100, 200)] + \
               [("tri", f) for f in (10, 50, 100)] + \
               [("sq", f) for f in (10, 50, 100)]
FIELDS = ["shape", "f_set", "f_actual", "amp_pct", "vref", "fs", "n",
          "f_meas", "f_fft", "vpp_meas", "vrms_ac", "mean_v", "noise_rms", "snr_db",
          "f_ref", "vpp_ref"]


def open_device(args):
    if args.sim:
        return DAQ(FakeDevice())
    if not args.port:
        sys.exit("error: give --port (e.g. COM3 or /dev/ttyACM0) or use --sim")
    return DAQ.open_serial(args.port, args.baud)


def print_measurements(m):
    if m["samples_per_period"] < 20:
        print("  WARNING: only %.1f samples per period; amplitude/noise of non-sine shapes degrade "
              "below ~20 (raise --rate or lower the signal frequency)" % m["samples_per_period"])
    kind = "sine-like" if m["sine_like"] else "non-sinusoidal (noise via 2nd difference)"
    print("  waveform        : %s" % kind)
    print("  frequency       : %.4f Hz   (zero-crossing %.4f, FFT %.4f, fit %.4f)"
          % (m["frequency"], m["freq_zc"], m["freq_fft"], m["freq_fit"]))
    print("  peak-to-peak    : %.4f V" % m["vpp"])
    print("  RMS (AC / total): %.4f V / %.4f V" % (m["vrms_ac"], m["vrms_total"]))
    print("  mean (DC)       : %.4f V" % m["mean_v"])
    print("  noise (approx.) : %.2f mV rms   SNR %.1f dB" % (m["noise_rms"] * 1e3, m["snr_db"]))


def cmd_gen(args):
    dev = open_device(args)
    f = dev.generate(args.shape, args.freq, args.amp, args.duty)
    print("device programmed %s at %.4f Hz" % (args.shape, f))


def capture_once(dev, fs, seconds, vref):
    dev.set_rate(fs)
    n = int(dev.fs * seconds)
    counts = dev.capture(n)
    return dev.fs, counts_to_volts(counts, vref)


def cmd_capture(args):
    dev = open_device(args)
    if args.shape:
        dev.generate(args.shape, args.freq, args.amp, args.duty)
    fs, v = capture_once(dev, args.rate, args.seconds, args.vref)
    m = analyze(v, fs, shape=args.shape)
    print("captured %d samples at %.2f S/s" % (len(v), fs))
    print_measurements(m)
    if args.csv:
        with open(args.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "volts"])
            for i, x in enumerate(v):
                w.writerow(["%.6f" % (i / fs), "%.5f" % x])
    if args.png:
        plot_capture(v, fs, m, args.png)
        print("saved", args.png)


def plot_capture(v, fs, m, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = np.arange(len(v)) / fs
    fig, (a, b) = plt.subplots(2, 1, figsize=(9, 6))
    show = min(len(v), int(max(3 / max(m["frequency"], 1e-9), 0.05) * fs))
    a.plot(t[:show], v[:show], ".-", lw=0.8, ms=3)
    a.set_xlabel("time (s)"); a.set_ylabel("volts"); a.grid(alpha=0.3)
    a.set_title("f=%.3f Hz  Vpp=%.3f V  Vrms(AC)=%.3f V  noise=%.1f mV"
                % (m["frequency"], m["vpp"], m["vrms_ac"], m["noise_rms"] * 1e3))
    x = v - np.mean(v)
    spec = np.abs(np.fft.rfft(x * np.hanning(len(x)))) / len(x)
    b.semilogy(np.fft.rfftfreq(len(x), 1 / fs), spec + 1e-9)
    b.set_xlabel("frequency (Hz)"); b.set_ylabel("magnitude"); b.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)


def cmd_live(args):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    dev = open_device(args)
    dev.set_rate(args.rate)
    dev.command("START")
    n = int(dev.fs * args.window)
    buf = np.full(n, np.nan)
    fig, ax = plt.subplots(figsize=(9, 4))
    (line,) = ax.plot(np.arange(n) / dev.fs, buf)
    ax.set_ylim(0, args.vref); ax.set_xlabel("time (s)"); ax.set_ylabel("volts"); ax.grid(alpha=0.3)
    txt = ax.text(0.01, 0.97, "", transform=ax.transAxes, va="top", family="monospace")

    def update(_):
        nonlocal buf
        s = dev.stream_chunks()
        if s:
            v = counts_to_volts(s, args.vref)
            buf = np.concatenate([buf, v])[-n:]
            line.set_ydata(buf)
            ok = buf[~np.isnan(buf)]
            if len(ok) > dev.fs * 0.5:
                m = analyze(ok, dev.fs)
                txt.set_text("f=%.2f Hz  Vpp=%.3f V  Vrms=%.3f V  noise=%.1f mV"
                             % (m["frequency"], m["vpp"], m["vrms_ac"], m["noise_rms"] * 1e3))
        return line, txt

    anim = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        dev.command("STOP")
    return anim


def cmd_sweep(args):
    """Run the standard test sweep: generate, capture, analyse, record. Loopback wiring:
    filter output (or D9) -> A0. Fill f_ref / vpp_ref later from an external instrument."""
    dev = open_device(args)
    rows = []
    only = set(args.only.split(",")) if args.only else None
    for shape, f in SWEEP_POINTS:
        if only and shape not in only:
            continue
        amp = 80.0
        dev.off()
        f_act = dev.generate(shape, f, amp if shape != "sq" else None)
        fs, v = capture_once(dev, args.rate, args.seconds, args.vref)
        m = analyze(v, fs, shape=shape)
        rows.append({
            "shape": shape, "f_set": f, "f_actual": "%.5f" % f_act,
            "amp_pct": amp if shape != "sq" else 100, "vref": args.vref, "fs": "%.3f" % fs,
            "n": len(v), "f_meas": "%.5f" % m["frequency"], "f_fft": "%.5f" % m["freq_fft"],
            "vpp_meas": "%.5f" % m["vpp"], "vrms_ac": "%.5f" % m["vrms_ac"],
            "mean_v": "%.5f" % m["mean_v"], "noise_rms": "%.6f" % m["noise_rms"],
            "snr_db": "%.2f" % m["snr_db"], "f_ref": "", "vpp_ref": ""})
        print("%-3s %7.2f Hz -> measured %.4f Hz, Vpp %.4f V" % (shape, f, m["frequency"], m["vpp"]))
    dev.off()
    new = not (args.append and os.path.exists(args.out))
    with open(args.out, "w" if new else "a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerows(rows)
    print("wrote", args.out)


def cmd_report(args):
    """Error table: measured vs expected (programmed) or vs external reference if filled in.
    Expected sine Vpp includes the RC filter's passband droop; triangle/square are compared to
    the ideal swing (the filter also rounds triangle peaks, so a small negative error is normal)."""
    rows = list(csv.DictReader(open(args.csv)))
    print("%-4s %9s %11s %9s | %9s %10s %9s" %
          ("shape", "f_ref(Hz)", "f_meas(Hz)", "f_err(%)", "Vpp_ref(V)", "Vpp_meas", "Vpp_err(%)"))
    out = []
    for r in rows:
        f_ref = float(r["f_ref"]) if r["f_ref"] else float(r["f_actual"])
        amp_expected = float(r["vref"]) * float(r["amp_pct"]) / 100.0
        if r["shape"] == "sin":
            amp_expected *= ladder_gain(f_ref, args.r, args.c)
        vpp_ref = float(r["vpp_ref"]) if r["vpp_ref"] else amp_expected
        f_err = 100 * (float(r["f_meas"]) - f_ref) / f_ref
        v_err = 100 * (float(r["vpp_meas"]) - vpp_ref) / vpp_ref
        out.append((r["shape"], f_ref, float(r["f_meas"]), f_err, vpp_ref, float(r["vpp_meas"]), v_err))
        print("%-4s %9.3f %11.4f %9.3f | %9.3f %10.4f %9.2f" % out[-1])
    fe = np.array([o[3] for o in out]); ve = np.array([o[6] for o in out])
    print("frequency error: mean |e| %.3f %%, max |e| %.3f %%" % (np.mean(abs(fe)), np.max(abs(fe))))
    print("Vpp error      : mean |e| %.2f %%, max |e| %.2f %%" % (np.mean(abs(ve)), np.max(abs(ve))))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port"); p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--sim", action="store_true", help="use the built-in firmware emulator")
    p.add_argument("--vref", type=float, default=5.0, help="measured Vcc/AREF in volts")
    sub = p.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("gen"); g.add_argument("shape", choices=["sin", "tri", "sq"])
    g.add_argument("freq", type=float); g.add_argument("--amp", type=float); g.add_argument("--duty", type=float)
    g.set_defaults(fn=cmd_gen)

    c = sub.add_parser("capture"); c.add_argument("--rate", type=float, default=2000)
    c.add_argument("--seconds", type=float, default=2.0)
    c.add_argument("--shape", choices=["sin", "tri", "sq"]); c.add_argument("--freq", type=float, default=50)
    c.add_argument("--amp", type=float); c.add_argument("--duty", type=float)
    c.add_argument("--csv"); c.add_argument("--png"); c.set_defaults(fn=cmd_capture)

    l = sub.add_parser("live"); l.add_argument("--rate", type=float, default=2000)
    l.add_argument("--window", type=float, default=2.0); l.set_defaults(fn=cmd_live)

    s = sub.add_parser("sweep"); s.add_argument("--rate", type=float, default=2000)
    s.add_argument("--seconds", type=float, default=2.0)
    s.add_argument("--out", default="hardware_measurements.csv")
    s.add_argument("--only", help="comma list of shapes to run, e.g. sin,tri or sq")
    s.add_argument("--append", action="store_true"); s.set_defaults(fn=cmd_sweep)

    r = sub.add_parser("report"); r.add_argument("csv")
    r.add_argument("--r", type=float, default=2200.0, help="filter resistor (ohm)")
    r.add_argument("--c", type=float, default=47e-9, help="filter capacitor (F)")
    r.set_defaults(fn=cmd_report)

    args = p.parse_args(argv)
    if args.cmd == "report":
        return cmd_report(args)
    return args.fn(args)


if __name__ == "__main__":
    main()
