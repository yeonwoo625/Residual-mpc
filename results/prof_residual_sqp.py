#!/usr/bin/env python3
"""
residual MPC solve 시간 구간 분해 + SQP 반복 횟수 스윕 (offline 재현 스크립트).

results/solvetime_summary.py 에 박아 넣은 숫자를 만들어 낸 계측이다.
소켓/Frenet 변환 없이 솔버만 반복 호출하므로 주행 중 값보다 약 20% 낮게 나온다.

실행 (서버와 동일한 환경이 필요하다):

  HOST=/home/vilab/CarMaker/mpc_host
  mkdir -p $HOST/run_prof && cd $HOST/run_prof
  export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
  source $HOST/venv/bin/activate
  export ACADOS_SOURCE_DIR=$HOST/acados
  export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
  export PYTHONPATH=/home/vilab/CarMaker/mpc_docker/mpc:$ACADOS_SOURCE_DIR/interfaces/acados_template
  export REFERENCE_PATH=$HOST/hockenheim_waypoints_1lap.npy
  export RESIDUAL_MODEL_PATH=/home/vilab/CarMaker/mpc_docker/mpc/residual_model_nomass_s0.pt
  python3 /home/vilab/CarMaker/mpc_docker/results/prof_residual_sqp.py

별도 디렉터리($HOST/run_prof)에서 돌려야 한다. $HOST/run 에서 돌리면 서버가 쓰는
c_generated_code 를 다시 빌드한다.
"""
import os
import sys
import time
import numpy as np

sys.path.insert(0, "/home/vilab/CarMaker/mpc_docker/mpc")
os.environ.setdefault("RESIDUAL_FF", "1")

from reference_utils import (                              # noqa: E402
    compute_path_spline, compute_kappa_spline_for_acados)
from mpc_solver_residual import MPCSolverResidual          # noqa: E402

ITERS = [1, 2, 3, 5, 10, 100]
N_WARM, N_TIME, N_STATE = 20, 100, 40


def build(sqp_iter):
    os.environ["SQP_ITER"] = str(sqp_iter)
    import importlib
    import mpc_solver_residual
    importlib.reload(mpc_solver_residual)
    w = np.load(os.environ["REFERENCE_PATH"])
    pi = compute_path_spline(np.asarray(w, dtype=float))
    co, kn = compute_kappa_spline_for_acados(pi["dense_s"], pi["kappa"], 3)
    mpc = mpc_solver_residual.MPCSolverResidual(
        coeffs=co, knots=kn, model_path=os.environ["RESIDUAL_MODEL_PATH"],
        Tf=2.0, N=20, target_speed=10.0, global_path_length=pi["s_total"])
    mpc.update_path(pi["dense_s"], pi["kappa"])
    return mpc


def timing(mpc):
    """solve() 총 시간과 l4acados 내부 타이머(1회 반복 기준)."""
    x0 = np.array([10.0, 0.05, 0.0, 10.0, 0.1, 0.0])
    for _ in range(N_WARM):
        mpc.solve(x0)
    t = []
    for _ in range(N_TIME):
        t0 = time.perf_counter()
        mpc.solve(x0)
        t.append((time.perf_counter() - t0) * 1e3)
    c = mpc.controller

    def med(name):
        a = np.asarray(getattr(c, name, []))[:max(c.num_iter, 1)]
        a = a[a > 0]
        return np.median(a) * 1e3 if a.size else float("nan")

    return (np.median(t), med("time_preparation"), med("time_residual"),
            med("time_nominal"), med("time_feedback"))


def controls(mpc):
    """고정된 무작위 상태 40개에서의 제어 명령 — 반복 횟수 간 해 비교용."""
    rng = np.random.default_rng(0)
    out = []
    for _ in range(N_STATE):
        x0 = np.array([rng.uniform(50, 3000), rng.uniform(-0.6, 0.6),
                       rng.uniform(-0.1, 0.1), rng.uniform(8, 12),
                       rng.uniform(-0.2, 0.5), rng.uniform(-0.1, 0.1)])
        for _ in range(3):
            mpc.solve(x0)
        D, d, _ = mpc.solve(x0)
        out.append([D, d])
    return np.array(out)


def main():
    rows, u = [], {}
    for it in ITERS:
        mpc = build(it)
        rows.append((it,) + timing(mpc))
        u[it] = controls(mpc)

    print("\n=== solve 시간 구간 분해 (median ms) ===")
    print(f"{'SQP':>5}{'total':>9}{'prep':>8}{'잔차NN':>9}{'적분기':>8}{'QP':>8}")
    for it, tot, pr, rs, nm, fb in rows:
        print(f"{it:>5d}{tot:>9.2f}{pr:>8.2f}{rs:>9.2f}{nm:>8.2f}{fb:>8.2f}")

    ref = u[max(ITERS)]
    print(f"\n=== {max(ITERS)}회 해 대비 조향 명령 차이 ({N_STATE}개 상태) ===")
    print(f"{'SQP':>5}{'max|dδ|(deg)':>15}{'rms|dδ|(deg)':>15}")
    for it in ITERS:
        d = np.abs(u[it][:, 1] - ref[:, 1]) * 180 / np.pi
        print(f"{it:>5d}{d.max():>15.4f}{np.sqrt((d ** 2).mean()):>15.4f}")


if __name__ == "__main__":
    main()
