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


def estimate_dt(s, v, v_min=3.0):
    """기록 행 간격 [s] 을 데이터에서 역산한다.

    궤적 파일에는 시간축이 없어 오랫동안 DT=0.1 을 가정했으나, 잔차 주행은
    solve 가 109 ms 라 제어 주기를 못 지키고 실제로는 ~6.4 Hz(0.155 s)로 돌았다
    (2026-09-02 확인, results/solvetime/ 참조). 한 스텝 진행거리 ds 와 속도 v 로
    dt = ds/v 를 구해 중앙값을 쓴다. 저속 구간은 ds/v 가 불안정해 제외한다.
    """
    ds = np.diff(s)
    vm = 0.5 * (v[1:] + v[:-1])
    m = vm > v_min
    return float(np.median(ds[m] / vm[m])) if m.sum() > 20 else DT


def evaluate(path, kf, v_target=None):
    a = np.load(path)
    w = np.where(np.diff(a[:, 0]) < -100)[0]      # 랩 경계
    a = a[:w[0] + 1] if len(w) else a
    s, n, alpha, v, delta = a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 5]
    corner = np.abs(kf(s)) >= KAPPA_CORNER
    dt = estimate_dt(s, v)

    # 종방향(x) 오차 = 진행거리 부족분. 목표속도로 갔을 때 대비 얼마나 뒤처졌나.
    # 정지 출발 가속 구간은 어떤 제어기든 뒤처지므로, 목표속도의 95% 에 처음
    # 도달한 시점을 기준으로 잡는다. 음수 = 뒤처짐.
    if v_target is None:
        x_mean = x_rms = x_final = np.nan
    else:
        i = int(np.argmax(v >= 0.95 * v_target))
        t = np.arange(len(s) - i) * dt
        xe = (s[i:] - s[i]) - v_target * t
        x_mean, x_rms, x_final = xe.mean(), np.sqrt((xe ** 2).mean()), xe[-1]
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
        # 헤딩오차 alpha (= yaw error, 경로 접선 대비 차체 방향). 전부 deg.
        a_mean     = np.rad2deg(np.abs(alpha).mean()),
        a_max      = np.rad2deg(np.abs(alpha).max()),
        a_corner   = (np.rad2deg(np.sqrt((alpha[corner] ** 2).mean()))
                      if corner.any() else np.nan),
        x_mean     = x_mean,
        x_rms      = x_rms,
        x_final    = x_final,
        lap_time   = (len(a) - 1) * dt,
        # 제어 주기를 데이터에서 역산해 쓴다. DT=0.1 을 고정하면 6.4 Hz 로 돈
        # 잔차 주행의 채터가 1.55배 과대평가된다.
        dt_est     = dt,
        rate_hz    = 1.0 / dt,
        chatter    = np.sqrt(np.mean((np.diff(delta) / dt) ** 2)),
        chatter_dt01 = np.sqrt(np.mean((np.diff(delta) / DT) ** 2)),   # 과거 값(참고)
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
