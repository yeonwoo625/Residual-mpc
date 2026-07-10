#!/usr/bin/env python3
"""
Analyze the R_DELTA sweep for first-order residual MPC.
Reports completion, lateral tracking |n|, and steering activity for each run,
and judges Case A (first-order salvaged: beats FF) vs Case B (feedforward-ized).

Usage:
    python3 analyze_R_sweep.py logs/R_*.npy [logs/ffR_*.npy ...]

dbglog (12 col): [s, n, alpha, v, D, delta, ||B dg/ddelta||, dn, da, dv, tgt_delta, status]
delta = col 5 (rad). Lap length ~2540 m.
"""
import sys, os, glob, numpy as np

R2D = 180.0 / np.pi
LAP = 2540.0
DONE_S = 2450.0     # reached near the finish => completed


def stats(path):
    a = np.load(path)
    s, n, delta = a[:, 0], a[:, 1], a[:, 5]
    # completed if s ever reaches near the finish (before any lap wrap)
    dec = np.where(np.diff(s) < -1.0)[0]
    lap1 = a[: dec[0] + 1] if len(dec) else a
    completed = lap1[:, 0].max() >= DONE_S
    an = np.abs(lap1[:, 1])
    dd = np.abs(np.diff(lap1[:, 5])) * R2D      # per-step steering change [deg]
    return dict(
        name=os.path.basename(path).replace(".npy", ""),
        N=len(lap1), smax=float(lap1[:, 0].max()), completed=completed,
        n_max=float(an.max()), n_p95=float(np.percentile(an, 95)),
        n_mean=float(an.mean()),
        dd_mean=float(dd.mean()), dd_max=float(dd.max()),
    )


def main():
    paths = []
    for pat in sys.argv[1:]:
        paths += sorted(glob.glob(pat))
    # always include the default-R FF baseline if present (context only)
    ff0 = "/home/vilab/CarMaker/mpc_host/logs/ff.npy"
    if os.path.exists(ff0) and ff0 not in paths:
        paths.append(ff0)
    if not paths:
        print("no logs found"); print(__doc__); sys.exit(1)

    rows = [stats(p) for p in paths]
    print(f"\n{'run':16s} {'done?':6s} {'s_max':>7s} {'|n|max':>7s} "
          f"{'|n|p95':>7s} {'|n|avg':>7s} {'dδ avg':>7s} {'dδ max':>7s}")
    print("-" * 72)
    for r in rows:
        print(f"{r['name']:16s} {'YES' if r['completed'] else 'no':6s} "
              f"{r['smax']:7.0f} {r['n_max']:7.2f} {r['n_p95']:7.2f} "
              f"{r['n_mean']:7.3f} {r['dd_mean']:7.2f} {r['dd_max']:7.2f}")

    # judgement: compare first-order runs that completed vs the FF at same R (ffR_*)
    fo = [r for r in rows if r["name"].startswith("R_") and r["completed"]]
    ff = [r for r in rows if r["name"].startswith("ffR_")]
    print("\n================ JUDGEMENT ================")
    if not fo:
        print("No first-order R-run completed yet. Increase R_DELTA further")
        print("or conclude first-order cannot be stabilized by R in this range.")
    else:
        best = min(fo, key=lambda r: r["n_p95"])
        print(f"first-order completes at {best['name']}: |n|p95={best['n_p95']:.2f}, "
              f"steer dδavg={best['dd_mean']:.2f} deg")
        if ff:
            ffm = min(ff, key=lambda r: r["n_p95"])
            print(f"feedforward @ same R ({ffm['name']}): |n|p95={ffm['n_p95']:.2f}, "
                  f"steer dδavg={ffm['dd_mean']:.2f} deg")
            if best["n_p95"] < ffm["n_p95"] * 0.95:
                print("=> Case A: first-order BEATS FF at this R -> SALVAGED (derivative helps). *")
            else:
                print("=> Case B: first-order NOT better than FF -> R just suppressed steering")
                print("   (feedforward-ized). Value-only remains the answer.")
        else:
            print("Run feedforward at the SAME R (ffR_<R*>.npy) for a fair comparison.")
    print("==========================================")


if __name__ == "__main__":
    main()
