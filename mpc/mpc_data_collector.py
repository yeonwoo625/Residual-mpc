#!/usr/bin/env python3
"""
TRUCK residual DATA COLLECTOR (acados NOMINAL MPC, NO rclpy).

Same unix-socket protocol as mpc_solver_server.py (so mpc_ros_relay.py works
unchanged), but instead of just controlling, it drives the truck with the
NOMINAL MPC and logs, every step:

    obs      = [n, alpha, v, D, delta, kappa]                   (6)
    residual = x_next_actual[:4] - x_next_nominal[:4]           (4) = [ds,dn,da,dv]

where x_next_nominal = predict_next_state(x_t, u_solver, dt)  (truck nominal),
and x_next_actual is the truck's real Frenet state one step (dt=0.1s) later.

Data is appended to OUT_NPZ (so you can run many sessions on different roads /
speeds to build coverage). Saved on relay disconnect and on Ctrl-C.

Env: REFERENCE_PATH, TARGET_SPEED, STEER_RATIO, OUT_NPZ, MPC_SOCK.
"""
import os
import sys
import signal
import socket
import struct
import numpy as np
from scipy.interpolate import make_interp_spline

from reference_utils import (
    compute_path_spline, compute_kappa_spline_for_acados, cartesian_to_frenet)
# USE_RESIDUAL=1 -> drive with FF residual MPC (DAgger 1st round): keeps corner
# |n| low so the steering dither survives (sc_n>0) in sharp corners. The residual
# TARGET stays exact = actual - nominal(actual control); policy only shifts the
# visited state distribution. Default 0 -> nominal MPC (original behavior).
USE_RESIDUAL = os.environ.get("USE_RESIDUAL", "0") == "1"
if USE_RESIDUAL:
    from mpc_solver_residual import MPCSolverResidual
else:
    from mpc_solver import MPCSolver
from dynamics_predict import predict_next_state

SOCK           = os.environ.get("MPC_SOCK", "/tmp/mpc_sock")
REFERENCE_PATH = os.environ["REFERENCE_PATH"]
TARGET_SPEED   = float(os.environ.get("TARGET_SPEED", "4.0"))
STEER_RATIO    = float(os.environ.get("STEER_RATIO", "10.0"))
OUT_NPZ        = os.environ.get(
    "OUT_NPZ", "/home/vilab/CarMaker/mpc_host/truck_dataset.npz")
MODEL_PATH     = os.environ.get(                # residual model for FF driving
    "RESIDUAL_MODEL_PATH",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "residual_model.pt"))
DT, Tf, N      = 0.1, 2.0, 20

# --- payload context (조건화 학습용) ----------------------------------------
# MASS, COG_X 를 둘 다 주면 obs 뒤에 [m, x_cog]를 붙여 8차원으로 기록한다.
# (적재조건별로 값 바꿔가며 여러 세션 수집 -> ConditionedResidualModel 학습용)
# 안 주면 기존처럼 6차원(무조건화).
_MASS  = os.environ.get("MASS", "")     # 적재 포함 총질량 [kg]
_COG_X = os.environ.get("COG_X", "")    # 정적 무게중심 x [m]
COND_CTX = (_MASS != "" and _COG_X != "")
CTX = np.array([float(_MASS), float(_COG_X)]) if COND_CTX else None

# --- excitation (off by default) -------------------------------------------
# OU-smoothed perturbation added to the applied control so the truck visits
# OFF-nominal states (n!=0, larger steering). Without it the residual model
# only ever sees near-centerline data and extrapolates badly off-distribution
# (-> the residual MPC went unstable & left the road). The nominal prediction
# uses the ACTUAL (perturbed) control, so the residual stays exact.
EXCITE_D     = float(os.environ.get("EXCITE_D", "0.0"))      # throttle pert. std
EXCITE_DELTA = float(os.environ.get("EXCITE_DELTA", "0.0"))  # steering pert. std [rad]
EXC_THETA    = float(os.environ.get("EXC_THETA", "0.85"))    # persistence 0..1 (~1/(1-θ) steps)
EXC_N_LIMIT  = float(os.environ.get("EXC_N_LIMIT", "1.0"))   # ease excitation off as |n|->this
EXC_VREF     = float(os.environ.get("EXC_VREF", "3.0"))      # scale pert. down above this speed
DELTA_MAX    = 0.4                                            # clip applied steering [rad]

REQ  = struct.Struct("dddd")   # x, y, yaw, v
RESP = struct.Struct("dddd")   # gas, brake, steering, gear

X_list, y_list = [], []        # accumulated obs / residual (this run)


def save():
    global X_list, y_list
    if not X_list:
        print("[collect] nothing to save", flush=True)
        return
    X = np.array(X_list)
    y = np.array(y_list)
    n_new = len(X)
    if os.path.exists(OUT_NPZ):
        d = np.load(OUT_NPZ)
        X = np.concatenate([d["X"], X], axis=0)
        y = np.concatenate([d["y"], y], axis=0)
    np.savez(OUT_NPZ, X=X, y=y)
    print(f"[collect] saved +{n_new} (total {len(X)}) -> {OUT_NPZ}", flush=True)
    X_list, y_list = [], []   # 저장한 건 비워서 이중 저장/재연결 중복 방지


def main():
    wpts = np.load(REFERENCE_PATH)
    pi = compute_path_spline(wpts.astype(float))
    co, kn = compute_kappa_spline_for_acados(pi["dense_s"], pi["kappa"], 3)
    ksp = make_interp_spline(pi["dense_s"], pi["kappa"], k=3)

    def kappa_func(s):
        sc = max(pi["dense_s"][0], min(s, pi["dense_s"][-1]))
        return float(ksp(sc))

    if USE_RESIDUAL:
        mpc = MPCSolverResidual(coeffs=co, knots=kn, model_path=MODEL_PATH, Tf=Tf, N=N,
                                target_speed=TARGET_SPEED, global_path_length=pi["s_total"])
        mpc.update_path(pi["dense_s"], pi["kappa"])
    else:
        mpc = MPCSolver(coeffs=co, knots=kn, Tf=Tf, N=N,
                        target_speed=TARGET_SPEED, global_path_length=pi["s_total"])
    exc_on = EXCITE_D > 0 or EXCITE_DELTA > 0
    print(f"[collect] {'RESIDUAL(FF)' if USE_RESIDUAL else 'NOMINAL'} MPC ready. path={pi['s_total']:.1f}m "
          f"target={TARGET_SPEED} m/s  excite={'ON' if exc_on else 'off'}"
          f"(D={EXCITE_D},δ={EXCITE_DELTA}) -> {OUT_NPZ}", flush=True)
    rng = np.random.default_rng()

    signal.signal(signal.SIGINT, lambda *a: (save(), sys.exit(0)))

    if os.path.exists(SOCK):
        os.remove(SOCK)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK)
    srv.listen(1)
    print(f"[collect] listening on {SOCK}", flush=True)

    D_prev = delta_prev = 0.0
    pending = None   # (obs, x_next_nominal) carried to the next step

    while True:
        conn, _ = srv.accept()
        print("[collect] relay connected", flush=True)
        D_prev = delta_prev = 0.0
        exc_D = exc_de = 0.0          # OU excitation state (per connection)
        pending = None
        try:
            while True:
                data = conn.recv(REQ.size)
                if len(data) < REQ.size:
                    break
                X_, Y_, yaw, v = REQ.unpack(data)
                s, n, alpha = cartesian_to_frenet(X_, Y_, yaw, pi)

                # ---- residual for the previous step (actual next is now) ----
                # (only if the actual next state is in-bounds; otherwise the
                #  truck left the road -> the residual would be garbage)
                if pending is not None:
                    obs_prev, xnom_prev = pending
                    if np.all(np.isfinite([s, n, alpha, v])) and abs(n) < 8.0:
                        X_list.append(obs_prev)
                        y_list.append(np.array([s, n, alpha, v]) - xnom_prev[:4])
                        if len(X_list) % 50 == 0:
                            print(f"[collect] {len(X_list)} samples (s={s:.1f} v={v:.2f})",
                                  flush=True)

                gas = brake = steering = 0.0
                gear = 1
                pending = None
                inb = (np.all(np.isfinite([s, n, alpha, v])) and abs(n) < 8.0
                       and 0.0 <= s < pi["s_total"] - 5.0)
                if inb:
                    x_t = np.array([s, n, alpha, v, D_prev, delta_prev])
                    D, delta, status = mpc.solve(x_t)
                    # only record / drive on a successful solve; otherwise the
                    # solver output is unreliable -> coast (0) and skip the sample
                    if status in (0, 2):
                        if exc_on:
                            # OU-smoothed perturbation, eased off near the edge
                            # AND scaled down with speed (lateral excursion ~ v),
                            # so the truck stays recoverable / the QP stays feasible.
                            sc = max(0.0, 1.0 - abs(n) / EXC_N_LIMIT)
                            sc *= min(1.0, EXC_VREF / max(v, 1.0))
                            exc_D  = EXC_THETA * exc_D  + rng.normal(0.0, EXCITE_D)
                            exc_de = EXC_THETA * exc_de + rng.normal(0.0, EXCITE_DELTA)
                            D     = float(np.clip(D + sc * exc_D, -1.0, 1.0))
                            delta = float(np.clip(delta + sc * exc_de, -DELTA_MAX, DELTA_MAX))
                        # rate of the ACTUAL applied control (exact even with
                        # excitation; equals the solver's u when excitation off)
                        u_app = np.array([(D - D_prev) / DT, (delta - delta_prev) / DT])
                        x_next_nom = predict_next_state(x_t, u_app, kappa_func, DT)
                        obs = np.array([n, alpha, v, D_prev, delta_prev, kappa_func(s)])
                        if COND_CTX:                       # 무게·CoG 조건 추가 -> 8차원
                            obs = np.concatenate([obs, CTX])
                        pending = (obs, x_next_nom)
                        D_prev, delta_prev = D, delta
                        if D >= 0:
                            gas = float(np.clip(D, 0.0, 1.0)); brake = 0.0
                        else:
                            gas = 0.0; brake = float(np.clip(-D, 0.0, 1.0))
                        steering = float(delta * STEER_RATIO)
                    else:
                        # solver failed -> hold the last applied control (coasting
                        # to 0 would steer straight and drift off a curve)
                        if D_prev >= 0:
                            gas = float(np.clip(D_prev, 0.0, 1.0)); brake = 0.0
                        else:
                            gas = 0.0; brake = float(np.clip(-D_prev, 0.0, 1.0))
                        steering = float(delta_prev * STEER_RATIO)

                conn.sendall(RESP.pack(gas, brake, steering, float(gear)))
        except (OSError, struct.error) as e:
            print(f"[collect] connection ended: {e}", flush=True)
        finally:
            conn.close()
            save()
            print("[collect] waiting for next relay connection", flush=True)


if __name__ == "__main__":
    main()
