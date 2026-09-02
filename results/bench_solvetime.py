#!/usr/bin/env python3
"""
MPC 해결 시간 벤치마크 — nominal vs residual(zero-order) vs residual(first-order).

주행 없이 측정한다. 실제 주행에서 기록한 상태(results/b1/v10_res.npy)를 그대로
재생하며 solve() 호출 시간만 잰다. 따라서 시뮬레이터·릴레이·통신 지연이 섞이지
않은 순수 솔버 시간이다.

세 구성을 한 프로세스에서 만들면 acados 코드 생성이 충돌하므로, 이 스크립트는
한 번에 한 구성만 측정하고(MODE 인자) 결과를 npz 로 남긴다. 세 개를 모두 돌리려면
run_all() 이 각 구성을 별도 작업 디렉터리의 서브프로세스로 실행한다.

사용:
    python3 results/bench_solvetime.py            # 세 구성 전부 (권장)
    python3 results/bench_solvetime.py nominal    # 하나만
"""
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
HOST = "/home/vilab/CarMaker/mpc_host"
WP = os.path.join(HOST, "hockenheim_waypoints_1lap.npy")
STATES = os.path.join(HERE, "b1", "v10_res.npy")     # 실제 주행 상태 재생
MODEL = os.path.join(ROOT, "mpc", "residual_model_nomass_s0.pt")
NSOLVE = 400          # 측정 횟수
NWARM = 30            # 워밍업(코드 캐시·JIT 안정화)
MODES = ["nominal", "zero_order", "first_order"]


def bench(mode):
    sys.path.insert(0, os.path.join(ROOT, "mpc"))
    sys.path.insert(0, os.getcwd())   # l4acados 가 cwd 의 c_generated_code 를 import 한다
    from reference_utils import compute_path_spline, compute_kappa_spline_for_acados
    pi = compute_path_spline(np.load(WP))
    coeffs, knots = compute_kappa_spline_for_acados(pi["dense_s"], pi["kappa"])
    kw = dict(Tf=2.0, N=20, target_speed=10.0, global_path_length=pi["dense_s"][-1])

    if mode == "nominal":
        from mpc_solver import MPCSolver
        mpc = MPCSolver(coeffs, knots, **kw)
    else:
        os.environ["RESIDUAL_MODEL_PATH"] = MODEL
        os.environ["RESIDUAL_SCALE"] = "0.3"
        os.environ["RESIDUAL_FF"] = "1" if mode == "zero_order" else "0"
        os.environ.pop("RESIDUAL_JAC_SCALE", None)
        from mpc_solver_residual import MPCSolverResidual
        mpc = MPCSolverResidual(coeffs, knots, MODEL, **kw)
        mpc.update_path(pi["dense_s"], pi["kappa"])

    a = np.load(STATES)
    w = np.where(np.diff(a[:, 0]) < -100)[0]
    a = a[:w[0] + 1] if len(w) else a
    a = a[a[:, 3] > 1.0]                                  # 정지 구간 제외
    idx = np.linspace(0, len(a) - 1, NSOLVE + NWARM).astype(int)

    ts = []
    for j, i in enumerate(idx):
        x0 = a[i, :6].astype(float)
        t0 = time.perf_counter()
        try:
            mpc.solve(x0)
        except Exception:
            continue
        dt = (time.perf_counter() - t0) * 1000.0          # ms
        if j >= NWARM:
            ts.append(dt)
    return np.array(ts)


def main():
    if len(sys.argv) > 1 and sys.argv[1] in MODES:
        mode = sys.argv[1]
        ts = bench(mode)
        out = os.path.join(HERE, f".bench_{mode}.npy")
        np.save(out, ts)
        print(f"[{mode}] n={len(ts)}  중앙값 {np.median(ts):.1f} ms  "
              f"평균 {ts.mean():.1f}  95% {np.percentile(ts,95):.1f}  최대 {ts.max():.1f}")
        return

    # 세 구성을 각각 별도 디렉터리의 서브프로세스로 (acados 코드생성 충돌 방지)
    res = {}
    for mode in MODES:
        d = os.path.join(HOST, "run", f"bench_{mode}")
        os.makedirs(d, exist_ok=True)
        print(f"\n=== {mode} 빌드·측정 중 (~1분) ===", flush=True)
        env = dict(os.environ)
        env["PYTHONPATH"] = os.path.join(ROOT, "mpc") + ":" + env.get("PYTHONPATH", "")
        r = subprocess.run([sys.executable, os.path.abspath(__file__), mode],
                           cwd=d, env=env, capture_output=True, text=True)
        tail = [l for l in r.stdout.splitlines() if l.startswith(f"[{mode}]")]
        print(tail[0] if tail else r.stdout[-400:] + r.stderr[-400:])
        f = os.path.join(HERE, f".bench_{mode}.npy")
        if os.path.exists(f):
            res[mode] = np.load(f); os.remove(f)

    if len(res) < 2:
        print("\n측정 실패 — 위 출력을 확인하라"); return
    from scipy.io import savemat
    print(f"\n{'구성':16s} {'중앙값':>9} {'평균':>8} {'95%':>8} {'최대':>8}  10Hz 예산(100ms)")
    print("-" * 68)
    for m in MODES:
        if m not in res: continue
        t = res[m]
        print(f"{m:16s} {np.median(t):8.1f}ms {t.mean():7.1f} {np.percentile(t,95):7.1f} "
              f"{t.max():7.1f}  {'충족' if np.percentile(t,95) < 100 else '초과'}")
    if "nominal" in res and "zero_order" in res:
        print(f"\n  residual(zero-order) / nominal = "
              f"{np.median(res['zero_order'])/np.median(res['nominal']):.2f}배")
    if "zero_order" in res and "first_order" in res:
        print(f"  first-order / zero-order       = "
              f"{np.median(res['first_order'])/np.median(res['zero_order']):.2f}배")
    savemat(os.path.join(HERE, "matlab", "fig_solvetime.mat"),
            {("t_" + m): res[m] for m in res} |
            {"labels": np.array([m for m in MODES if m in res], dtype=object),
             "budget_ms": 100.0, "n_solve": NSOLVE})
    print(f"\n저장: {os.path.join(HERE,'matlab','fig_solvetime.mat')}")


if __name__ == "__main__":
    main()
