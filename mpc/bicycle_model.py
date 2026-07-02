"""
Frenet-frame Dynamic Bicycle Model (Tesla Model 3)
Adapted from adeebislam8/carla_mpc_residual_learning, ROS dependencies removed,
obstacle constraints removed for simplification.

state x = [s, n, alpha, v, D, delta]    (6)
input u = [derD, derDelta]               (2)
"""
import math
import types
import numpy as np
from casadi import MX, vertcat, cos, sin, tan, tanh, Function

DEG2RAD = math.pi / 180.0


def bicycle_model(dt, coeff, knots, degree=3):
    """
    Args:
        dt     : time step [s] (Tf/N)
        coeff  : B-spline coefficients of kappa(s)  (from scipy make_interp_spline.c)
        knots  : B-spline knots of kappa(s)         (from scipy make_interp_spline.t)
        degree : spline degree (default 3)
    """
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Spatialbicycle_model"

    # CasADi B-spline of curvature kappa(s)
    kapparef_s = Function.bspline('kapparef_s', [knots], coeff, [degree], 1)

    # ---- Demo4AxleRigidTruck8x4_Actros (4143K 8x4/4) parameters ----
    # From vehicle file + data sheet: wheelbase 4.5 m, CoG x=4.35,
    # front-axle group center 6.6, rear-axle group center 2.275.
    m  = 30000.0       # Body.mass 29600 + wheel carriers etc.
    lf = 2.25          # CoG -> front axle group [m]
    lr = 2.07          # CoG -> rear axle group  [m]  (lf+lr = 4.32 ~ 4.5)
    C1 = lr / (lr + lf)
    C2 = 1 / (lr + lf) * 0.5

    # Longitudinal force F = (Cm1 - Cm2 v) D - Cr2 v^2 - Cr0 tanh(Cr3 v).
    # Identified on truck data (sysid_longitudinal.py, 2026-06-10, 13k samples,
    # v 0.5-7.9 m/s -> Δv RMS -38%, bias removed). Cr3 kept fixed (nonlinear).
    Cm1 = 2.8739e+04   # peak drive force [N]
    Cm2 = 1.4157e+03   # drive-force fade with speed
    Cr2 = 1.2778e+01   # aero drag
    Cr0 = 3.3817e+03   # rolling resistance
    Cr3 = 1.1e+01

    # ---- States ----
    s     = MX.sym("s")
    n     = MX.sym("n")
    alpha = MX.sym("alpha")
    v     = MX.sym("v")
    D     = MX.sym("D")
    delta = MX.sym("delta")
    x = vertcat(s, n, alpha, v, D, delta)

    # ---- Inputs (rates) ----
    derD     = MX.sym("derD")
    derDelta = MX.sym("derDelta")
    u = vertcat(derD, derDelta)

    # ---- xdot symbols ----
    sdot     = MX.sym("sdot")
    ndot     = MX.sym("ndot")
    alphadot = MX.sym("alphadot")
    vdot     = MX.sym("vdot")
    Ddot     = MX.sym("Ddot")
    deltadot = MX.sym("deltadot")
    xdot = vertcat(sdot, ndot, alphadot, vdot, Ddot, deltadot)

    # ---- Algebraic / parameters ----
    z = vertcat([])
    p = vertcat([])    # 장애물 제거 → 빈 vector

    # ---- Dynamics ----
    Fxd = (Cm1 - Cm2 * v) * D - Cr2 * v * v - Cr0 * tanh(Cr3 * v)
    a_long = Fxd / m

    # 원본 코드에 'delta = -delta' 있어 그대로 유지
    # delta_eff = -delta
    delta_eff = delta

    sdot_expr     = (v * cos(alpha + C1 * delta_eff)) / (1 - kapparef_s(s) * n)
    ndot_expr     = v * sin(alpha + C1 * delta_eff)
    alphadot_expr = v * C2 * delta_eff - kapparef_s(s) * sdot_expr
    vdot_expr     = a_long * cos(C1 * delta_eff)

    f_expl = vertcat(
        sdot_expr,
        ndot_expr,
        alphadot_expr,
        vdot_expr,
        derD,
        derDelta,
    )

    # ---- Constraint expressions ----
    a_lat = C2 * v * v * delta_eff + a_long * sin(C1 * delta_eff)
    constraint.expr = vertcat(a_long, a_lat, n, v, D, delta)

    # ---- Bounds ----
    # narrow margin: wide truck (2.49 m) on a normal lane -> keep it centred
    model.n_min = -1.0
    model.n_max =  1.0
    model.v_min = 0.0
    model.v_max = 150.0
    model.throttle_min = -1.0
    model.throttle_max =  1.0
    model.delta_min = -30 * DEG2RAD
    model.delta_max =  30 * DEG2RAD
    model.ddelta_min = -1.0
    model.ddelta_max =  1.0
    model.dthrottle_min = -1.0
    model.dthrottle_max =  1.0

    constraint.alat_min  = -5.0
    constraint.alat_max  =  5.0
    constraint.along_min = -10.0
    constraint.along_max =  10.0

    # ---- Initial condition ----
    model.x0 = np.array([0, 0, 0, 0, 0, 0])

    # ---- Pack model ----
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name

    return model, constraint