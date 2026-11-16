"""Signal analysis for ADC sample streams.

Measures frequency (zero-crossing and FFT), peak-to-peak amplitude, RMS, mean
and an approximate noise level from a sampled waveform.

Methods (see docs/DESIGN.md for the reasoning):
  frequency  : Schmitt-trigger zero crossings with linear interpolation, averaged over all
               complete periods; cross-checked with a Hann-windowed FFT (parabolic peak).
  Vpp        : sine-like signals -> least-squares sine fit amplitude x2 (noise-robust);
               other shapes -> mean per-period maximum minus mean per-period minimum
               (falls back to the 0.5th/99.5th percentile spread with < 2 periods).
  RMS        : AC RMS (mean removed) and total RMS.
  noise      : sine-like -> RMS of the residual after a 4-parameter sine fit;
               other shapes -> robust (MAD) estimate from the second difference.
               It is APPROXIMATE and includes ADC quantisation noise.
"""
import numpy as np
from scipy.optimize import minimize_scalar

ADC_BITS = 10


def counts_to_volts(counts, vref=5.0, bits=ADC_BITS):
    """Convert raw ADC counts to volts (Arduino convention: V = counts * Vref / 2^bits)."""
    return np.asarray(counts, dtype=float) * vref / (1 << bits)


def _schmitt_rising_crossings(ac, hyst):
    """Return fractional sample positions of rising zero crossings with hysteresis."""
    out = []
    armed = False
    for i in range(1, len(ac)):
        if ac[i - 1] < -hyst:
            armed = True
        if armed and ac[i - 1] < 0.0 <= ac[i]:
            frac = -ac[i - 1] / (ac[i] - ac[i - 1])
            out.append(i - 1 + frac)
            armed = False
    return np.array(out)


def freq_zero_crossing(v, fs):
    """Frequency from rising zero crossings of the mean-removed signal. NaN if < 2 crossings."""
    ac = v - np.mean(v)
    span = np.percentile(ac, 99) - np.percentile(ac, 1)
    if span <= 0:
        return float("nan")
    c = _schmitt_rising_crossings(ac, 0.1 * span)
    if len(c) < 2:
        return float("nan")
    period = (c[-1] - c[0]) / (len(c) - 1)
    return fs / period


def vpp_per_period(v):
    """Peak-to-peak as mean(per-period max) - mean(per-period min); averaging over periods
    suppresses the upward bias that noise puts on a single global max. NaN if < 2 periods."""
    ac = v - np.mean(v)
    span = np.percentile(ac, 99) - np.percentile(ac, 1)
    if span <= 0:
        return float("nan")
    c = _schmitt_rising_crossings(ac, 0.1 * span)
    if len(c) < 3:
        return float("nan")
    idx = np.ceil(c).astype(int)
    maxs = [v[a:b].max() for a, b in zip(idx[:-1], idx[1:]) if b > a]
    mins = [v[a:b].min() for a, b in zip(idx[:-1], idx[1:]) if b > a]
    return float(np.mean(maxs) - np.mean(mins))


def freq_fft(v, fs):
    """Frequency from the FFT peak (Hann window, parabolic interpolation of log-magnitude)."""
    x = v - np.mean(v)
    n = len(x)
    spec = np.abs(np.fft.rfft(x * np.hanning(n)))
    spec[0] = 0.0
    k = int(np.argmax(spec[1:]) + 1)
    if 1 <= k < len(spec) - 1 and spec[k - 1] > 0 and spec[k + 1] > 0:
        a, b, c = np.log(spec[k - 1]), np.log(spec[k]), np.log(spec[k + 1])
        denom = a - 2 * b + c
        delta = 0.5 * (a - c) / denom if denom != 0 else 0.0
    else:
        delta = 0.0
    return (k + delta) * fs / n


def _sine_fit_at(v, t, f):
    """Linear least squares for [cos, sin, 1] at fixed f. Returns (amp, offset, residual)."""
    w = 2 * np.pi * f * t
    m = np.column_stack([np.cos(w), np.sin(w), np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(m, v, rcond=None)
    res = v - m @ coef
    return float(np.hypot(coef[0], coef[1])), float(coef[2]), res


def sine_fit(v, fs, f0):
    """4-parameter sine fit: refine frequency around f0, return (f, amp, offset, residual)."""
    t = np.arange(len(v)) / fs
    periods = max(1.0, f0 * len(v) / fs)
    half = f0 * min(1e-3, 0.3 / periods)

    def cost(f):
        return float(np.sum(_sine_fit_at(v, t, f)[2] ** 2))

    r = minimize_scalar(cost, bounds=(f0 - half, f0 + half), method="bounded",
                        options={"xatol": f0 * 1e-9})
    f = float(r.x)
    amp, off, res = _sine_fit_at(v, t, f)
    return f, amp, off, res


def triangle_fit_vpp(v, fs, f):
    """Peak-to-peak from a least-squares fit of an ideal triangle wave v = c + A*tri(f*t + phi).

    Fitting the waveform itself (rather than scaling the fundamental or reading sample maxima)
    is immune to harmonics aliasing onto the fundamental when fs/f is an integer, and to peaks
    falling between samples. Returns 2*|A|."""
    t = np.arange(len(v)) / fs

    def tri(x):
        return 1.0 - 4.0 * np.abs((x % 1.0) - 0.5)

    def solve(phi):
        m = np.column_stack([tri(f * t + phi), np.ones_like(t)])
        coef, *_ = np.linalg.lstsq(m, v, rcond=None)
        return coef, float(np.sum((v - m @ coef) ** 2))

    grid = np.linspace(0, 1, 65)[:-1]
    best = grid[int(np.argmin([solve(p)[1] for p in grid]))]
    r = minimize_scalar(lambda p: solve(p)[1], bounds=(best - 1 / 64, best + 1 / 64),
                        method="bounded", options={"xatol": 1e-7})
    return float(2 * abs(solve(r.x)[0][0]))


def noise_second_difference(v):
    """White-noise sigma from the second difference, Var(d2) = 6 sigma^2.

    Samples where the signal itself bends (corners of a triangle, edges of a square) give large
    d2 outliers; they are rejected at 3.5 robust-sigma before taking a plain standard deviation
    (the MAD x 1.4826 rule alone over-reads non-Gaussian noise such as ADC quantisation)."""
    d2 = v[2:] - 2 * v[1:-1] + v[:-2]
    med = np.median(d2)
    robust = 1.4826 * np.median(np.abs(d2 - med))
    keep = np.abs(d2 - med) <= 3.5 * max(robust, 1e-12)
    return float(np.std(d2[keep]) / np.sqrt(6.0))


def analyze(v, fs, sine_threshold=0.05, shape=None):
    """Analyse a voltage record `v` sampled at `fs` Hz. Returns a dict of measurements.

    shape: optional hint.
      'tri' : a triangle's sharp peaks fall between samples when there are few samples per
              period, so Vpp comes from a least-squares fit of an ideal triangle wave
              (triangle_fit_vpp) instead of from the sample maxima.
      'sq'  : Vpp = median of the upper plateau - median of the lower plateau, which is not
              biased upward by noise the way a per-period maximum is."""
    v = np.asarray(v, dtype=float)
    mean = float(np.mean(v))
    ac = v - mean
    res = {
        "n": len(v),
        "fs": fs,
        "mean_v": mean,
        "vrms_ac": float(np.sqrt(np.mean(ac ** 2))),
        "vrms_total": float(np.sqrt(np.mean(v ** 2))),
        "vpp_pct": float(np.percentile(v, 99.5) - np.percentile(v, 0.5)),
        "freq_zc": freq_zero_crossing(v, fs),
        "freq_fft": freq_fft(v, fs),
    }
    f0 = res["freq_zc"] if np.isfinite(res["freq_zc"]) else res["freq_fft"]
    f_fit, amp, off, resid = sine_fit(v, fs, f0)
    resid_rms = float(np.std(resid))
    sine_rms = amp / np.sqrt(2)
    res["sine_like"] = bool(sine_rms > 0 and resid_rms / sine_rms < sine_threshold)
    res["freq_fit"] = f_fit
    res["samples_per_period"] = fs / f_fit
    res["vpp_fit"] = 2 * amp
    res["residual_rms"] = resid_rms
    if res["sine_like"]:
        res["frequency"] = f_fit
        res["vpp"] = 2 * amp
        res["noise_rms"] = resid_rms
        res["snr_db"] = float(20 * np.log10(sine_rms / resid_rms)) if resid_rms > 0 else float("inf")
    else:
        res["frequency"] = res["freq_zc"] if np.isfinite(res["freq_zc"]) else res["freq_fft"]
        vp = vpp_per_period(v)
        res["vpp"] = vp if np.isfinite(vp) else res["vpp_pct"]
        if shape in ("tri", "triangle"):
            res["vpp"] = triangle_fit_vpp(v, fs, f_fit)
        elif shape in ("sq", "square"):
            mid = 0.5 * (v.max() + v.min())
            res["vpp"] = float(np.median(v[v > mid]) - np.median(v[v < mid]))
        res["noise_rms"] = noise_second_difference(v)
        res["snr_db"] = float(20 * np.log10(res["vrms_ac"] / res["noise_rms"])) if res["noise_rms"] > 0 else float("inf")
    return res
