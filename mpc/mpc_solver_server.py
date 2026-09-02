#!/usr/bin/env python3
"""
Residual MPC solver SERVER (acados + l4acados + torch, NO rclpy/DDS).

Listens on a unix-domain socket. For each request it receives the vehicle
state (x, y, yaw, v) as 4 float64, runs the MPC, and replies with the control
(gas, brake, steering[rad], gear) as 4 float64.

Run together with mpc_ros_relay.py (which owns rclpy/DDS). Splitting the two
avoids a segfault from fast-DDS and acados living in one process.

Env: REFERENCE_PATH (.npy waypoints), TARGET_SPEED, STEER_RATIO, MPC_SOCK.
"""
import os
import socket
import struct
import time
import numpy as np

from reference_utils import (
    compute_path_spline, compute_kappa_spline_for_acados, cartesian_to_frenet)

# USE_RESIDUAL=1 -> truck nominal + learned residual (needs a TRUCK-trained
# residual model; the shipped one is CARLA-car and invalid for the truck).
# Default 0 -> pure truck NOMINAL MPC (honest baseline until residual retrained).
USE_RESIDUAL = os.environ.get("USE_RESIDUAL", "0") == "1"
if USE_RESIDUAL:
    from mpc_solver_residual import MPCSolverResidual
else:
    from mpc_solver import MPCSolver

SOCK           = os.environ.get("MPC_SOCK", "/tmp/mpc_sock")
REFERENCE_PATH = os.environ.get("REFERENCE_PATH", "")
TARGET_SPEED   = float(os.environ.get("TARGET_SPEED", "4.0"))
STEER_RATIO    = float(os.environ.get("STEER_RATIO", "10.0"))
Tf, N          = 2.0, 20
MODEL_PATH     = os.environ.get(
    "RESIDUAL_MODEL_PATH",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "residual_model.pt"))

REQ  = struct.Struct("dddd")   # x, y, yaw, v
RESP = struct.Struct("dddd")   # gas, brake, steering, gear


def build(wpts):
    pi = compute_path_spline(np.asarray(wpts, dtype=float))
    co, kn = compute_kappa_spline_for_acados(pi["dense_s"], pi["kappa"], 3)
    if USE_RESIDUAL:
        mpc = MPCSolverResidual(
            coeffs=co, knots=kn, model_path=MODEL_PATH, Tf=Tf, N=N,
            target_speed=TARGET_SPEED, global_path_length=pi["s_total"])
        mpc.update_path(pi["dense_s"], pi["kappa"])
    else:
        mpc = MPCSolver(
            coeffs=co, knots=kn, Tf=Tf, N=N,
            target_speed=TARGET_SPEED, global_path_length=pi["s_total"])
    return mpc, pi


def main():
    if not (REFERENCE_PATH and os.path.exists(REFERENCE_PATH)):
        raise SystemExit("REFERENCE_PATH not set / missing")
    wpts = np.load(REFERENCE_PATH)
    mode = "RESIDUAL" if USE_RESIDUAL else "NOMINAL"
    print(f"[server] building {mode} MPC from {len(wpts)} waypoints "
          f"(target {TARGET_SPEED} m/s, steer_ratio {STEER_RATIO}) ...", flush=True)
    mpc, pi = build(wpts)
    kmax = float(np.max(np.abs(pi["kappa"])))
    print(f"[server] MPC ready. path={pi['s_total']:.1f}m max_kappa={kmax:.4f}", flush=True)

    D_prev = delta_prev = 0.0

    # 궤적 로깅 (LOG_TRAJ=path 주면 매 solve의 [s,n,alpha,v,D,delta]를 저장).
    # nominal vs residual 추종오차(|n|) 비교용. 연결 종료/Ctrl-C 때 저장.
    LOG_TRAJ = os.environ.get("LOG_TRAJ", "")
    traj = []
    print(f"[server] LOG_TRAJ = {LOG_TRAJ or '(미설정 -> 궤적 저장 안 함)'}", flush=True)

    def save_traj(quiet=False):
        # 저장이 조용히 스킵되면 주행을 통째로 날린다 -> 왜 안 됐는지 반드시 알린다.
        if not LOG_TRAJ:
            if not quiet:
                print("[server] LOG_TRAJ 미설정 -> 저장 안 함", flush=True)
            return
        if not traj:
            if not quiet:
                print("[server] 기록된 solve 0건 -> 저장할 궤적 없음", flush=True)
            return
        arr = np.array(traj)
        np.save(LOG_TRAJ, arr)
        if not quiet:
            print(f"[server] traj saved: {arr.shape} -> {LOG_TRAJ} "
                  f"(|n| mean={np.abs(arr[:,1]).mean():.3f} max={np.abs(arr[:,1]).max():.3f})",
                  flush=True)

    # solve 시간 계측 (LOG_SOLVETIME=path 를 주면 매 solve 의 ms 를 저장).
    # nominal vs residual 연산량 비교용. 미설정이면 아무 것도 하지 않는다.
    LOG_SOLVETIME = os.environ.get("LOG_SOLVETIME", "")
    solve_ms = []
    print(f"[server] LOG_SOLVETIME = {LOG_SOLVETIME or '(미설정)'}", flush=True)

    def save_solvetime():
        if not LOG_SOLVETIME or not solve_ms:
            return
        arr = np.array(solve_ms)
        np.save(LOG_SOLVETIME, arr)
        print(f"[server] solve time: n={len(arr)} median={np.median(arr):.1f}ms "
              f"mean={arr.mean():.1f} p95={np.percentile(arr,95):.1f} max={arr.max():.1f} "
              f"-> {LOG_SOLVETIME}", flush=True)

    import signal as _sig
    _sig.signal(_sig.SIGINT, lambda *a: (save_traj(), save_solvetime(), exit(0)))

    if os.path.exists(SOCK):
        os.remove(SOCK)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK)
    srv.listen(1)
    print(f"[server] listening on {SOCK}", flush=True)

    while True:
        conn, _ = srv.accept()
        print("[server] relay connected", flush=True)
        n_solve = 0
        try:
            while True:
                data = conn.recv(REQ.size)
                if len(data) < REQ.size:
                    break
                X, Y, yaw, v = REQ.unpack(data)
                s, n, alpha = cartesian_to_frenet(X, Y, yaw, pi)

                gas = brake = steering = 0.0
                gear = 1
                x0 = np.array([s, n, alpha, v, D_prev, delta_prev])
                ok = (np.all(np.isfinite(x0)) and abs(n) < 8.0
                      and 0.0 <= s < pi["s_total"] - 5.0)
                if ok:
                    # solve 내부(l4acados/acados/모델)가 off-distribution 상태에서
                    # 예외를 던질 수 있다. 잡아서 마지막 제어를 유지 -> 서버 안 죽음.
                    try:
                        _t0 = time.perf_counter()
                        D, delta, status = mpc.solve(x0)
                        if LOG_SOLVETIME:
                            solve_ms.append((time.perf_counter() - _t0) * 1000.0)
                    except Exception as e:
                        if n_solve % 10 == 0:
                            print(f"[server] solve failed ({e}); holding last control",
                                  flush=True)
                        D, delta, status = D_prev, delta_prev, -1
                    D_prev, delta_prev = D, delta
                    if D >= 0:
                        gas = float(np.clip(D, 0.0, 1.0)); brake = 0.0
                    else:
                        gas = 0.0; brake = float(np.clip(-D, 0.0, 1.0))
                    steering = float(delta * STEER_RATIO)
                    if LOG_TRAJ:
                        traj.append([s, n, alpha, v, D, delta])
                        if len(traj) % 200 == 0:
                            save_traj(quiet=True)   # kill/창닫기로도 안 날아가게
                    if n_solve % 10 == 0:
                        print(f"[server] s={s:6.1f} n={n:+.2f} a={np.rad2deg(alpha):+.1f} "
                              f"v={v:.2f}/{TARGET_SPEED} D={D:+.3f} "
                              f"delta={np.rad2deg(delta):+.1f}", flush=True)
                    n_solve += 1
                elif n_solve % 10 == 0:
                    print(f"[server] skip (oob): s={s:.1f} n={n:.2f} v={v:.2f}", flush=True)
                    n_solve += 1

                conn.sendall(RESP.pack(gas, brake, steering, float(gear)))
        except (OSError, struct.error) as e:
            print(f"[server] connection ended: {e}", flush=True)
        finally:
            conn.close()
            save_traj()
            save_solvetime()
            print("[server] relay disconnected, waiting for next", flush=True)


if __name__ == "__main__":
    main()
