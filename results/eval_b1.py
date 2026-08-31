#!/usr/bin/env python3
"""
B1 (nominal 가중치 튜닝) 주행 평가 — 논문 지표와 동일하게 계산.

  python3 results/eval_b1.py <traj.npy> [<traj2.npy> ...]
  python3 results/eval_b1.py /home/vilab/CarMaker/mpc_host/b1_*.npy

지표
  급코너 RMS |n|  : R <= 50 m (|kappa| >= 0.02) 구간의 횡오차 RMS  ← 튜닝 목적함수
  평균 |n|        : 전 구간
  RMS(dδ)         : 조향 변화율 = 채터 지표. residual MPC 값을 넘으면 탈락
  완주            : 한 바퀴(2540 m) 돌았는지

첫 랩만 사용한다(랩 경계에서 s가 급감하는 지점으로 자름).
"""
import os
import sys
import numpy as np
from scipy.interpolate import make_interp_spline

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "mpc"))
from reference_utils import compute_path_spline           # noqa: E402

WP  = "/home/vilab/CarMaker/mpc_host/hockenheim_waypoints_1lap.npy"
DT  = 0.1
KAPPA_CORNER = 1.0 / 50.0          # R <= 50 m


def _kappa_fn():
    pi = compute_path_spline(np.load(WP))
    sp = make_interp_spline(pi["dense_s"], pi["kappa"], k=3)
    lo, hi = pi["dense_s"][0], pi["dense_s"][-1]
    return lambda s: sp(np.clip(s, lo, hi))


def evaluate(path, kf):
    a = np.load(path)
    w = np.where(np.diff(a[:, 0]) < -100)[0]      # 랩 경계
    a = a[:w[0] + 1] if len(w) else a
    s, n, v, delta = a[:, 0], a[:, 1], a[:, 3], a[:, 5]
    corner = np.abs(kf(s)) >= KAPPA_CORNER
    return dict(
        name       = os.path.basename(path),
        steps      = len(a),
        dist       = s.max() - s.min(),
        done       = (s.max() - s.min()) > 2400,
        v_mean     = v.mean(),
        n_mean     = np.abs(n).mean(),
        n_rms      = np.sqrt((n ** 2).mean()),
        n_max      = np.abs(n).max(),
        corner_rms = np.sqrt((n[corner] ** 2).mean()) if corner.any() else np.nan,
        chatter    = np.sqrt(np.mean((np.diff(delta) / DT) ** 2)),
    )


def main():
    files = sys.argv[1:]
    if not files:
        sys.exit(__doc__)
    kf = _kappa_fn()
    rows = [evaluate(f, kf) for f in files]
    print(f"{'파일':26s} {'완주':>4} {'속도':>6} {'급코너RMS':>10} {'평균|n|':>8} "
          f"{'RMS|n|':>8} {'최대|n|':>8} {'조향채터':>9}")
    print("-" * 88)
    for r in rows:
        print(f"{r['name'][:26]:26s} {'O' if r['done'] else 'X':>4} {r['v_mean']:6.2f} "
              f"{r['corner_rms']:10.3f} {r['n_mean']:8.3f} {r['n_rms']:8.3f} "
              f"{r['n_max']:8.3f} {r['chatter']:9.4f}")
    if len(rows) > 1:
        ok = [r for r in rows if r["done"]]
        if ok:
            best = min(ok, key=lambda r: r["corner_rms"])
            print(f"\n최저 급코너 RMS: {best['name']}  ({best['corner_rms']:.3f})")
            print("※ 우승자가 스윕 범위의 끝값이면 그 방향으로 더 탐색할 것.")
            print("※ 조향채터가 residual MPC 값보다 크면 탈락시킬 것.")


if __name__ == "__main__":
    main()
