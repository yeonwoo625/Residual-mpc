"""
Frenet bicycle NOMINAL dynamics for one-step prediction, used to compute the
residual (actual - nominal) during data collection.

Parameters MUST stay in sync with bicycle_model.py
(Demo4AxleRigidTruck8x4_Actros). If you change one, change both.
"""
import numpy as np

# ---- Demo4AxleRigidTruck8x4_Actros (match bicycle_model.py) ----
M  = 30000.0
LF = 2.25
LR = 2.07
C1 = LR / (LR + LF)
C2 = 1.0 / (LR + LF) * 0.5

# identified on truck data (sysid_longitudinal.py 2026-06-10, Δv RMS -38%)
CM1 = 2.8739e+04
CM2 = 1.4157e+03
CR2 = 1.2778e+01
CR0 = 3.3817e+03
CR3 = 1.1e+01


def f_nominal(x, u, kappa_func):
    """state x=[s,n,alpha,v,D,delta], input u=[derD,derDelta]."""
    s, n, alpha, v, D, delta = x
    derD, derDelta = u

    Fxd = (CM1 - CM2 * v) * D - CR2 * v * v - CR0 * np.tanh(CR3 * v)
    a_long = Fxd / M

    kappa = float(kappa_func(s))
    sdot     = (v * np.cos(alpha + C1 * delta)) / (1.0 - kappa * n + 1e-9)
    ndot     = v * np.sin(alpha + C1 * delta)
    alphadot = v * C2 * delta - kappa * sdot
    vdot     = a_long * np.cos(C1 * delta)
    return np.array([sdot, ndot, alphadot, vdot, derD, derDelta])


def predict_next_state(x, u, kappa_func, dt):
    """One-step Euler integration of the nominal model."""
    return x + dt * f_nominal(x, u, kappa_func)
