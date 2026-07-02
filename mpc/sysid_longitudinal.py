#!/usr/bin/env python3
"""
Re-identify the truck longitudinal force coefficients (Cm1, Cm2, Cr2, Cr0)
from a collected residual dataset.

Force model (linear in Cm1,Cm2,Cr2,Cr0; Cr3 & geometry kept fixed):
    Fxd  = (Cm1 - Cm2*v)*D - Cr2*v^2 - Cr0*tanh(Cr3*v)
    vdot = (Fxd / M) * cos(C1*delta)

The dataset stores obs=[n,alpha,v,D,delta,kappa] and y=[ds,dn,dalpha,dv].
We reconstruct the truck's real longitudinal accel from the residual:
    vdot_actual = vdot_nominal_old(v,D,delta) + dv/dt
    Fxd_actual  = vdot_actual * M / cos(C1*delta)
and least-squares fit:
    Fxd_actual = [D, -v*D, -v^2, -tanh(Cr3*v)] . [Cm1, Cm2, Cr2, Cr0]

NOTE: needs a WIDE speed range (collect at several TARGET_SPEED values),
otherwise the regression is ill-conditioned.

Usage: sysid_longitudinal.py [dataset.npz]
Env:   VMIN (ignore samples below this speed, default 0.5 m/s)
"""
import os
import sys
import numpy as np
import dynamics_predict as dp     # current (old) coeffs + geometry

DT   = 0.1
DATA = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/vilab/CarMaker/mpc_host/truck_dataset.npz"
VMIN = float(os.environ.get("VMIN", "0.5"))


def main():
    d = np.load(DATA)
    X, y = d["X"], d["y"]
    v, D, delta = X[:, 2], X[:, 3], X[:, 4]
    dv = y[:, 3]

    M, C1, CR3 = dp.M, dp.C1, dp.CR3
    cosd = np.cos(C1 * delta)

    # reconstruct absolute longitudinal accel from the residual + old nominal
    Fxd_old  = (dp.CM1 - dp.CM2 * v) * D - dp.CR2 * v**2 - dp.CR0 * np.tanh(CR3 * v)
    vdot_old = (Fxd_old / M) * cosd
    vdot_act = vdot_old + dv / DT
    Fxd_act  = vdot_act * M / cosd

    m = np.isfinite(Fxd_act) & (v > VMIN)
    v_, D_, Fxd_ = v[m], D[m], Fxd_act[m]
    p10, p90 = np.percentile(v_, [10, 90])   # robust spread (ignores accel tail)
    print(f"[sysid] {m.sum()}/{len(v)} samples (v>{VMIN}); "
          f"v [{v_.min():.2f},{v_.max():.2f}] (p10-p90 {p10:.2f}-{p90:.2f})  "
          f"D [{D_.min():.2f},{D_.max():.2f}]")

    A = np.stack([D_, -v_ * D_, -v_**2, -np.tanh(CR3 * v_)], axis=1)
    theta, *_ = np.linalg.lstsq(A, Fxd_, rcond=None)
    Cm1, Cm2, Cr2, Cr0 = theta

    r2 = 1 - np.sum((Fxd_ - A @ theta) ** 2) / np.sum((Fxd_ - Fxd_.mean()) ** 2)
    cond = float(np.linalg.cond(A))

    # The metric that actually matters: does the new nominal reduce the 1-step
    # speed error Δv?  (R^2 on force is low because the truck powertrain is
    # geared/nonlinear -- that structure is the residual model's job, not the
    # nominal's.)  NOTE: assumes the dataset was collected with the CURRENT
    # bicycle_model coeffs (dp.*); run sysid on fresh data after each change.
    Fxd_new  = (Cm1 - Cm2 * v) * D - Cr2 * v**2 - Cr0 * np.tanh(CR3 * v)
    vdot_new = (Fxd_new / M) * cosd
    dv_new   = dv + DT * (vdot_old - vdot_new)
    fin = np.isfinite(dv) & np.isfinite(dv_new) & (v > VMIN)
    rms_old = float(np.sqrt(np.mean(dv[fin] ** 2)))
    rms_new = float(np.sqrt(np.mean(dv_new[fin] ** 2)))
    improve = 100 * (1 - rms_new / rms_old) if rms_old > 0 else 0.0

    print(f"\n[sysid] force-fit R^2={r2:.3f} (low is normal: geared powertrain), cond={cond:.1e}")
    print(f"[sysid] Δv RMS (1-step speed err): {rms_old:.4f} -> {rms_new:.4f}  ({improve:+.1f}%)")
    print(f"  {'coeff':5s} {'old':>13s} {'new':>13s}")
    for nm, o, n in [('Cm1', dp.CM1, Cm1), ('Cm2', dp.CM2, Cm2),
                     ('Cr2', dp.CR2, Cr2), ('Cr0', dp.CR0, Cr0)]:
        print(f"  {nm:5s} {o:13.4f} {n:13.4f}")
    print(f"  {'Cr3':5s} {dp.CR3:13.4f} {dp.CR3:13.4f}   (fixed)")

    nonphys = (Cm1 <= 0) or (Cm2 < 0) or (Cr2 < 0) or (Cr0 < 0)
    narrow  = (p90 - p10) < 1.5
    if nonphys or narrow or improve < 2.0:
        print("\n[sysid] *** DO NOT apply: ***")
        if narrow:  print(f"    - speed too concentrated (p10-p90 = {p10:.2f}-{p90:.2f} m/s)")
        if nonphys: print("    - non-physical coefficient (negative)")
        if improve < 2.0: print(f"    - no meaningful Δv improvement ({improve:+.1f}%)")
        print("    => collect more / wider speeds and re-run.")
    else:
        print(f"\n[sysid] OK ({improve:+.1f}% Δv) -> update CM1/CM2/CR2/CR0 in BOTH "
              "bicycle_model.py and dynamics_predict.py.")


if __name__ == "__main__":
    main()
