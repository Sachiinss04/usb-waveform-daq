#!/usr/bin/env python3
"""Monte Carlo validation of analysis.py against KNOWN ground truth.

Signals with exactly known frequency, Vpp, RMS and noise are passed through a simulated
10-bit / 5 V ADC (quantisation + gaussian noise) and analysed. The error statistics show how
accurate the measurement *algorithms* are. This is SIMULATED data: it does not measure the
real hardware (that is what `daq_app.py sweep` + `report` are for).

Usage: python synthetic_validation.py [--trials 30] [--out ../results]
"""
import argparse
import csv
import os

import numpy as np

from analysis import analyze, counts_to_volts

VREF, BITS = 5.0, 10
LSB = VREF / 2 ** BITS
FS = 2000.0
NOISE_V = 0.002                       # 2 mV rms analogue noise before the ADC
FREQS = (5, 10, 20, 50, 100, 200)
SHAPES = ("sine", "triangle", "square")
DURATIONS = (2.0, 0.5)


def make_signal(shape, f, vpp, n, rng):
    t = np.arange(n) / FS
    ph = (t * f + rng.random()) % 1.0
    if shape == "sine":
        w, rms = np.sin(2 * np.pi * ph), 1 / np.sqrt(2)
    elif shape == "triangle":
        w, rms = 1 - 4 * np.abs(ph - 0.5), 1 / np.sqrt(3)
    else:
        w, rms = np.where(ph < 0.5, 1.0, -1.0), 1.0
    return 2.5 + 0.5 * vpp * w, rms * 0.5 * vpp


def adc(v, rng):
    v = v + rng.normal(0, NOISE_V, len(v))
    return counts_to_volts(np.clip(np.round(v / VREF * 2 ** BITS), 0, 2 ** BITS - 1), VREF, BITS)


def run(trials, seed=1):
    rng = np.random.default_rng(seed)
    rows = []
    noise_true = np.sqrt(NOISE_V ** 2 + LSB ** 2 / 12)
    for dur in DURATIONS:
        n = int(FS * dur)
        for shape in SHAPES:
            for f in FREQS:
                fe, ve, re_, ne, vn = [], [], [], [], []
                for k in range(trials):
                    # half the trials use the EXACT nominal frequency: fs/f is then an integer
                    # (both come from the same crystal on real hardware), the worst case for
                    # harmonic aliasing; the other half detune by +-2 %
                    f_true = f if k % 2 == 0 else f * (1 + rng.uniform(-0.02, 0.02))
                    vpp = rng.uniform(0.5, 4.5)
                    clean, rms_true = make_signal(shape, f_true, vpp, n, rng)
                    meas = adc(clean, rng)
                    m = analyze(meas, FS, shape={"triangle": "tri", "square": "sq"}.get(shape))
                    vn.append(100 * (analyze(meas, FS)["vpp"] - vpp) / vpp)
                    fe.append(100 * (m["frequency"] - f_true) / f_true)
                    ve.append(100 * (m["vpp"] - vpp) / vpp)
                    re_.append(100 * (m["vrms_ac"] - rms_true) / rms_true)
                    ne.append(m["noise_rms"] / noise_true)
                fe, ve, re_, ne, vn = map(np.array, (fe, ve, re_, ne, vn))
                rows.append({
                    "duration_s": dur, "shape": shape, "f_nominal_hz": f,
                    "periods_in_record": round(f * dur, 1),
                    "freq_err_mean_abs_pct": np.mean(abs(fe)), "freq_err_max_abs_pct": np.max(abs(fe)),
                    "vpp_err_mean_abs_pct": np.mean(abs(ve)), "vpp_err_max_abs_pct": np.max(abs(ve)),
                    "vpp_err_max_abs_pct_no_hint": np.max(abs(vn)),
                    "vrms_err_mean_abs_pct": np.mean(abs(re_)), "vrms_err_max_abs_pct": np.max(abs(re_)),
                    "noise_est_over_true_median": np.median(ne)})
    return rows


def write_outputs(rows, out):
    os.makedirs(out, exist_ok=True)
    keys = list(rows[0].keys())
    with open(os.path.join(out, "synthetic_validation.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 5) if isinstance(v, float) else v) for k, v in r.items()})
    md = ["| record | shape | f (Hz) | periods | freq err mean/max (%) | Vpp err mean/max (%) "
          "| Vpp max err, no shape hint (%) | Vrms err mean/max (%) | noise est / true |",
          "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        md.append("| %.1f s | %s | %d | %.0f | %.4f / %.4f | %.3f / %.3f | %.3f | %.3f / %.3f | %.2f |" % (
            r["duration_s"], r["shape"], r["f_nominal_hz"], r["periods_in_record"],
            r["freq_err_mean_abs_pct"], r["freq_err_max_abs_pct"],
            r["vpp_err_mean_abs_pct"], r["vpp_err_max_abs_pct"], r["vpp_err_max_abs_pct_no_hint"],
            r["vrms_err_mean_abs_pct"], r["vrms_err_max_abs_pct"], r["noise_est_over_true_median"]))
    open(os.path.join(out, "synthetic_validation.md"), "w").write("\n".join(md) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(13, 4))
    for dur, ls in zip(DURATIONS, ("-", "--")):
        for shape, c in zip(SHAPES, ("C0", "C1", "C2")):
            sel = [r for r in rows if r["duration_s"] == dur and r["shape"] == shape]
            x = [r["f_nominal_hz"] for r in sel]
            lab = "%s, %.1f s" % (shape, dur)
            ax[0].loglog(x, [max(r["freq_err_max_abs_pct"], 1e-6) for r in sel], ls, c=c, marker="o", label=lab)
            ax[1].semilogx(x, [r["vpp_err_max_abs_pct"] for r in sel], ls, c=c, marker="o", label=lab)
            ax[2].semilogx(x, [r["noise_est_over_true_median"] for r in sel], ls, c=c, marker="o", label=lab)
    ax[0].set_title("Frequency error (max |%|)"); ax[1].set_title("Vpp error (max |%|)")
    ax[2].set_title("Noise estimate / true (median)"); ax[2].axhline(1, c="k", lw=0.5)
    for a in ax:
        a.set_xlabel("signal frequency (Hz)"); a.grid(alpha=0.3, which="both")
    ax[0].legend(fontsize=7)
    fig.suptitle("Analysis accuracy on SIMULATED 10-bit ADC data (fs=2 kS/s, 2 mV rms noise)")
    fig.tight_layout(); fig.savefig(os.path.join(out, "synthetic_validation.png"), dpi=130)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results"))
    a = ap.parse_args()
    rows = run(a.trials)
    write_outputs(rows, a.out)
    print(open(os.path.join(a.out, "synthetic_validation.md")).read())
