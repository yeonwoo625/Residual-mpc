#!/usr/bin/env python3
"""
Re-express a residual dataset against the CURRENT (re-identified) nominal model
WITHOUT re-driving.

Only the longitudinal force coeffs (Cm1,Cm2,Cr2,Cr0) changed, so only Δv moves:
    Δv_new = Δv_old + dt * (vdot_old(OLD coeffs) - vdot_new(current coeffs))
Δs, Δn, Δalpha are unaffected (s/n/alpha dynamics don't use Cm/Cr); M, C1 (geometry)
are unchanged too.

Usage: renominalize.py IN.npz OUT.npz
Env:   OLD_CM1/OLD_CM2/OLD_CR2/OLD_CR0/OLD_CR3 = coeffs the data was COLLECTED with
       (defaults = the pre-sysid truck estimates).
"""
import os
import sys
import numpy as np
import dynamics_predict as dp   # CURRENT (re-identified) coeffs

DT = 0.1
IN, OUT = sys.argv[1], sys.argv[2]

OLD = dict(CM1=4.0e4, CM2=2.0e2, CR2=3.4, CR0=2.1e3, CR3=11.0)
for k in OLD:
    OLD[k] = float(os.environ.get("OLD_" + k, OLD[k]))
NEW = dict(CM1=dp.CM1, CM2=dp.CM2, CR2=dp.CR2, CR0=dp.CR0, CR3=dp.CR3)


def vdot(v, D, delta, c):
    Fxd = (c["CM1"] - c["CM2"]*v)*D - c["CR2"]*v**2 - c["CR0"]*np.tanh(c["CR3"]*v)
    return (Fxd / dp.M) * np.cos(dp.C1 * delta)


def main():
    d = np.load(IN)
    X, y = d["X"], d["y"]
    v, D, delta = X[:, 2], X[:, 3], X[:, 4]

    y2 = y.copy()
    y2[:, 3] = y[:, 3] + DT * (vdot(v, D, delta, OLD) - vdot(v, D, delta, NEW))

    r_old = float(np.sqrt(np.mean(y[:, 3] ** 2)))
    r_new = float(np.sqrt(np.mean(y2[:, 3] ** 2)))
    np.savez(OUT, X=X, y=y2)
    print(f"[renom] {len(X)} samples: Δv RMS {r_old:.4f} -> {r_new:.4f} "
          f"({100*(1-r_new/r_old):+.1f}%)  (Δn,Δα unchanged) -> {OUT}")


if __name__ == "__main__":
    main()
