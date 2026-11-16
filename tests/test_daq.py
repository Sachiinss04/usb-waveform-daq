import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "python"))
from analysis import analyze, counts_to_volts, noise_second_difference  # noqa: E402
from daqlink import DAQ, FakeDevice, StreamDecoder, ladder_gain         # noqa: E402

FS = 2000.0


def sig(shape, f, vpp, n=4000, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    ph = (np.arange(n) / FS * f) % 1.0
    w = {"sine": np.sin(2 * np.pi * ph), "tri": 1 - 4 * np.abs(ph - 0.5),
         "sq": np.where(ph < 0.5, 1.0, -1.0)}[shape]
    return 2.5 + 0.5 * vpp * w + rng.normal(0, noise, n)


def test_sine_frequency_amplitude_rms():
    m = analyze(sig("sine", 50.0, 3.0), FS)
    assert m["sine_like"]
    assert m["frequency"] == pytest.approx(50.0, rel=1e-5)
    assert m["vpp"] == pytest.approx(3.0, rel=1e-4)
    assert m["vrms_ac"] == pytest.approx(3.0 / (2 * np.sqrt(2)), rel=2e-3)
    assert m["mean_v"] == pytest.approx(2.5, abs=1e-3)


def test_noise_estimate_sine_and_triangle():
    for shape in ("sine", "tri"):
        m = analyze(sig(shape, 20.0, 3.0, noise=0.01, seed=3), FS, shape=shape)
        assert m["noise_rms"] == pytest.approx(0.01, rel=0.15)


@pytest.mark.parametrize("phase", [0.0, 0.13, 0.37])
def test_triangle_fit_is_exact_even_with_integer_fs_over_f(phase):
    # 200 Hz at 2 kS/s = exactly 10 samples/period: harmonics alias onto the fundamental and
    # peaks fall between samples. A fitted triangle model is unaffected.
    ph = (np.arange(4000) / FS * 200.0 + phase) % 1.0
    v = 2.5 + 2.0 * (1 - 4 * np.abs(ph - 0.5))
    assert analyze(v, FS, shape="tri")["vpp"] == pytest.approx(4.0, rel=1e-4)
    if phase:
        assert abs(analyze(v, FS)["vpp"] - 4.0) > 0.3   # documents why the shape hint exists


def test_square_levels():
    m = analyze(sig("sq", 50.0, 4.0, noise=0.003), FS, shape="sq")
    assert not m["sine_like"]
    assert m["vpp"] == pytest.approx(4.0, rel=5e-3)
    assert m["frequency"] == pytest.approx(50.0, rel=1e-3)


def test_freq_methods_agree():
    m = analyze(sig("sine", 37.3, 2.0, noise=0.005, seed=1), FS)
    assert m["freq_zc"] == pytest.approx(37.3, rel=2e-4)
    assert m["freq_fft"] == pytest.approx(37.3, rel=2e-3)


def test_counts_to_volts():
    assert counts_to_volts(1023) == pytest.approx(5.0 * 1023 / 1024)
    assert counts_to_volts(512, vref=4.8) == pytest.approx(2.4)


def test_decoder_roundtrip_with_interleaved_text():
    def enc(c):
        return bytes([0x80 | (c >> 5), c & 0x1F])
    vals = [0, 1, 31, 32, 511, 512, 1023]
    stream = b"OK START fs=2000.000\n" + b"".join(enc(c) for c in vals) + b"OK STOP drops=0\n"
    s, lines = StreamDecoder().feed(stream)
    assert s == vals
    assert lines == ["OK START fs=2000.000", "OK STOP drops=0"]


def test_decoder_resynchronises_after_lost_byte():
    def enc(c):
        return bytes([0x80 | (c >> 5), c & 0x1F])
    stream = enc(100) + enc(200)[:1] + enc(300) + enc(400)   # second sample lost its low byte
    s, _ = StreamDecoder().feed(stream)
    assert s == [100, 300, 400]


def test_decoder_works_when_bytes_arrive_one_at_a_time():
    d = StreamDecoder()
    out = []
    for b in bytes([0x80 | (700 >> 5), 700 & 0x1F]):
        out += d.feed(bytes([b]))[0]
    assert out == [700]


def test_emulated_device_end_to_end():
    dev = DAQ(FakeDevice(noise_v=0.002, seed=5))
    assert dev.set_rate(2000) == pytest.approx(2000.0)
    f = dev.generate("sin", 50, amp=80)
    assert f == pytest.approx(50.0, abs=1e-3)
    m = analyze(counts_to_volts(dev.capture(4000)), dev.fs)
    assert m["frequency"] == pytest.approx(50.0, rel=1e-4)
    assert m["vpp"] == pytest.approx(4.0 * ladder_gain(50.0), rel=2e-3)   # 80 % of 5 V, minus filter droop


def test_rate_is_quantised_to_timer_divisor():
    dev = DAQ(FakeDevice())
    assert dev.set_rate(1500) == pytest.approx(250000 / 167)   # nearest integer divisor
    assert dev.set_rate(100000) == pytest.approx(5000.0)       # clamped to the maximum rate
    assert dev.set_rate(100) == pytest.approx(250000 / 256)    # clamped to the minimum rate


def test_device_error_is_reported():
    dev = DAQ(FakeDevice())
    with pytest.raises(RuntimeError):
        dev.command("BOGUS")
