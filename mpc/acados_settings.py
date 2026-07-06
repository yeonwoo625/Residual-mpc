"""
acados OCP setup for Frenet dynamic bicycle MPC.
Adapted from adeebislam8/carla_mpc_residual_learning, simplified.
"""
import os
import math
import numpy as np
import scipy.linalg
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from bicycle_model import bicycle_model

DEG2RAD = math.pi / 180.0


def acados_settings(Tf, N, coeffs, knots, degree=3,
                    json_file="acados_ocp.json"):
    """
    Args:
        Tf     : prediction horizon [s]
        N      : horizon discretization steps
        coeffs : B-spline coefficients of kappa(s)
        knots  : B-spline knots of kappa(s)
        degree : spline degree
        json_file : output json (acados internal)
    """
    ocp = AcadosOcp()
    dt = Tf / N
    print("[acados_settings] building model...")
    model, constraint = bicycle_model(dt, coeffs, knots, degree)
    print("[acados_settings] model built")

    # ---- AcadosModel wrap ----
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x    = model.x
    model_ac.xdot = model.xdot
    model_ac.u    = model.u
    model_ac.z    = model.z
    model_ac.p    = model.p
    model_ac.name = model.name
    ocp.model = model_ac
    model_ac.con_h_expr = constraint.expr

    # ---- Dimensions ----
    nx = model.x.size()[0]      # 6
    nu = model.u.size()[0]      # 2
    ny   = nx + nu              # 8
    ny_e = nx                   # 6

    nh  = constraint.expr.shape[0]   # 6 (a_long, a_lat, n, v, D, delta)
    nsh = 3                          # slack: along, alat, n_max만
    ns  = nsh

    ocp.dims.N = N

    # ---- Cost ----
    ocp.cost.cost_type   = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    unscale = N / Tf

    Q = np.diag([
        1e-5,   # s
        1e-4,   # n
        1e-5,   # alpha
        1e-3,   # v        ← 0.0 -> 1e-3
        0.0,    # D
        0.0,    # delta
    ])
    # 조향/스로틀 변화율 벌점. R_DELTA를 키우면 조향이 매끄러워짐(잔차 chatter 억제).
    # 환경변수로 튜닝 가능 (기본은 기존 값 2e-4).
    R = np.eye(nu)
    R[0, 0] = float(os.environ.get("R_D", "2e-4"))       # derD (throttle rate)
    R[1, 1] = float(os.environ.get("R_DELTA", "2e-4"))   # derDelta (steering rate)
    print(f"[acados] R_D={R[0,0]:.1e}  R_DELTA={R[1,1]:.1e}", flush=True)

    Qe = np.diag([
        5e-5,   # s
        1e-4,   # n
        1e-5,   # alpha
        0.0,    # v
        0.0,    # D
        0.0,    # delta
    ])

    ocp.cost.W   = unscale * scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Qe / unscale

    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[6, 0] = 1.0
    Vu[7, 1] = 1.0
    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    ocp.cost.yref   = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # ---- Input bounds ----
    ocp.constraints.lbu = np.array([model.dthrottle_min, model.ddelta_min])
    ocp.constraints.ubu = np.array([model.dthrottle_max, model.ddelta_max])
    ocp.constraints.idxbu = np.array([0, 1])

    # ---- Nonlinear constraints (a_long, a_lat, n, v, D, delta) ----
    ocp.constraints.lh = np.array([
        constraint.along_min,
        constraint.alat_min,
        model.n_min,
        model.v_min,
        model.throttle_min,
        model.delta_min,
    ])
    ocp.constraints.uh = np.array([
        constraint.along_max,
        constraint.alat_max,
        model.n_max,
        model.v_max,
        model.throttle_max,
        model.delta_max,
    ])

    # ---- Slack only on first 3 (along, alat, n_max) ----
    slack_L1 = np.array([1e-6, 1e-6, 1e1])
    slack_L2 = np.array([1e-6, 1e-6, 1e1])
    ocp.cost.zl = slack_L1
    ocp.cost.zu = slack_L1
    ocp.cost.Zl = slack_L2
    ocp.cost.Zu = slack_L2

    ocp.constraints.lsh = np.zeros(nsh)
    ocp.constraints.ush = np.zeros(nsh)
    ocp.constraints.idxsh = np.array([0, 1, 2])

    # ---- Initial condition ----
    ocp.constraints.x0 = model.x0

    # ---- Solver options ----
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver       = "PARTIAL_CONDENSING_HPIPM"
    # NLP_SOLVER=SQP + NLP_MAX_ITER>1 -> full SQP (RTI 민감성 검증용). 기본 SQP_RTI.
    ocp.solver_options.nlp_solver_type = os.environ.get("NLP_SOLVER", "SQP_RTI")
    ocp.solver_options.nlp_solver_max_iter = int(os.environ.get("NLP_MAX_ITER", "1"))
    ocp.solver_options.hessian_approx  = "GAUSS_NEWTON"
    ocp.solver_options.levenberg_marquardt = 1e-3
    print(f"[acados] nlp_solver={ocp.solver_options.nlp_solver_type} "
          f"max_iter={ocp.solver_options.nlp_solver_max_iter}", flush=True)

    acados_solver = AcadosOcpSolver(ocp, json_file=json_file)
    print("[acados_settings] solver created")

    return constraint, model, acados_solver