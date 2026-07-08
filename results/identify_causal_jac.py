#!/usr/bin/env python3
"""
Identify the CAUSAL steering Jacobian d(residual)/d(delta) from decorrelated
(steering-excited) data, and compare it against the confounded closed-loop value.

Model-free (Frisch-Waugh-Lovell partial regression, closed-form OLS) so the
result cannot be dismissed as "the MLP was built badly".

Usage:
    python3 identify_causal_jac.py EXCITED.npz [CLOSEDLOOP.npz]

Each npz: X (N,6+) = [n, alpha, v, D_prev, delta_prev, kappa, (mass, cog)]
          y (N,4)   = [ds, dn, da, dv]   (residual = actual - nominal)
delta index = 4. Targets of interest: dn (y[:,1]) and da (y[:,2]).
"""
import sys, numpy as np

DELTA = 4                      # column of delta_prev in X
OTHERS = [0, 1, 2, 3, 5]       # n, alpha, v, D_prev, kappa (all state cols but delta)
TGT = {"dn": 1, "da": 2}       # lateral residual channels


def _ols(A, b):
    A1 = np.c_[A, np.ones(len(A))]
    w, *_ = np.linalg.lstsq(A1, b, rcond=None)
    pred = A1 @ w
    ss = np.sum((b - np.mean(b)) ** 2)
    r2 = 1 - np.sum((b - pred) ** 2) / ss if ss > 0 else 0.0
    return w, pred, r2


def analyze(path):
    d = np.load(path)
    X, y = d["X"], d["y"]
    delta = X[:, DELTA]
    others = X[:, OTHERS]

    # ---- collinearity of delta w.r.t. the other states ----
    _, dhat, r2_d = _ols(others, delta)          # delta ~ others
    d_perp = delta - dhat                          # exogenous part of delta
    vif = 1.0 / max(1e-9, 1 - r2_d)
    print(f"\n=== {path}  (N={len(X)}) ===")
    print(f"  VIF(delta)      = {vif:8.1f}   (want < 5 for identification)")
    print(f"  delta_perp std  = {d_perp.std():8.4f} rad   (want >= 0.02)")
    print(f"  corr(dperp, n)  = {np.corrcoef(d_perp, X[:,0])[0,1]:+.3f}   "
          f"corr(dperp, kappa) = {np.corrcoef(d_perp, X[:,5])[0,1]:+.3f}  (want ~0)")

    # ---- Frisch-Waugh causal slope for each lateral channel ----
    out = {}
    for name, j in TGT.items():
        _, that, _ = _ols(others, y[:, j])        # target ~ others
        t_perp = y[:, j] - that
        denom = float(d_perp @ d_perp)
        slope = float(d_perp @ t_perp) / denom if denom > 0 else np.nan
        # std error of slope
        resid = t_perp - slope * d_perp
        dof = max(1, len(X) - len(OTHERS) - 2)
        se = np.sqrt(np.sum(resid**2) / dof / denom) if denom > 0 else np.nan
        # partial R2 (how much of target_perp the delta_perp explains)
        ss = np.sum((t_perp - t_perp.mean())**2)
        pr2 = 1 - np.sum(resid**2)/ss if ss > 0 else 0.0
        out[name] = (slope, se, pr2)
        print(f"  causal d({name})/d(delta) = {slope:+.3f} +- {se:.3f}"
              f"   (partial R2={pr2:.3f})")
    return out, vif, d_perp.std()


def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    exc, _, exc_dstd = analyze(sys.argv[1])
    cl = None
    if len(sys.argv) >= 3:
        cl, _, _ = analyze(sys.argv[2])

    print("\n================ DECISION ================")
    s_dn = exc["dn"][0]
    print(f"excited (decorrelated) causal d(dn)/d(delta) = {s_dn:+.3f}")
    if cl is not None:
        print(f"closed-loop (confounded) slope             = {cl['dn'][0]:+.3f}")
    print(f"closed-loop learned Jacobian (MLP, prior)    = -0.540 (destabilizing)")
    print("------------------------------------------")
    if abs(s_dn) < 0.15:
        print("=> Case 1: causal ~ 0  -> -0.54 was a CORRELATION artifact.")
        print("   Retrain residual on excited data, then test first-order (JAC_SCALE=1).")
        print("   If it completes -> first-order is SALVAGEABLE via active excitation.")
    elif s_dn < -0.3:
        print("=> Case 2: causal still strongly negative -> REAL physics (understeer).")
        print("   First-order over-aggression is structural, not a data artifact.")
        print("   -> zero-order (value-only) is principled; no real-truck retry needed.")
    else:
        print("=> Intermediate: partial cancellation. Inspect magnitude vs nominal gain")
        print("   (+0.57). If |causal| << 0.54, expect much milder instability.")
    print("==========================================")


if __name__ == "__main__":
    main()
