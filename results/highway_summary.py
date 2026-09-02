#!/usr/bin/env python3
"""
미학습 트랙 일반화 — Hockenheim 에서 학습, USA Highway No.1 에서 시험.

일반화 축이 '적재' 하나뿐이라 트랙 축을 추가했다. 잔차 모델은 **재학습하지 않고**
Hockenheim 에서 학습한 것(residual_model_nomass_s0.pt)을 그대로 사용했다.

결정적인 점: 이 도로는 차선 폭 3.5 m 이고 트럭 폭이 2.49 m 라 좌우 여유가
각 0.5 m 뿐이다. 즉 성능 차이가 '오차가 줄었다'가 아니라 **완주 가능 여부**로 나타난다.

  nominal  : 최대 횡오차 0.835 m -> 여유 초과, 625 m 에서 도로 이탈 (SIM_ABORT,
             "Vehicle leaves road ... TireNo=1")
  residual : 최대 0.213 m -> 931 m 전 구간 완주

곡률 검증: 이 경로의 κ 는 학습 데이터 분포(1~99%) 밖 비율이 0.0% 다. 즉 외삽이
아니라 순수한 경로 일반화다. 최소 회전반경은 Hockenheim 37.8 m vs Highway 91.8 m.

사용:  python3 results/highway_summary.py
출력:  표 + results/matlab/fig_highway.mat
"""
import os
import sys
import numpy as np
from scipy.io import savemat
from scipy.interpolate import make_interp_spline

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "mpc"))
from reference_utils import compute_path_spline               # noqa: E402

HW    = os.path.join(HERE, "highway")
OUT   = os.path.join(HERE, "matlab", "fig_highway.mat")
LANE_W, TRUCK_W = 3.5, 2.49
MARGIN = (LANE_W - TRUCK_W) / 2.0          # 좌우 여유 [m]


def lap1(path):
    a = np.load(path)
    w = np.where(np.diff(a[:, 0]) < -100)[0]
    return a[:w[0] + 1] if len(w) else a


def main():
    pi = compute_path_spline(np.load(os.path.join(HW, "highway_waypoints.npy")))
    kap = pi["kappa"]
    tr_k = np.load(os.path.join(HERE, "ablation", "hockenheim_nomass.npz"))["X"][:, 5]
    lo, hi = np.percentile(tr_k, 1), np.percentile(tr_k, 99)

    print("미학습 트랙: USA Highway No.1  (학습은 Hockenheim, 재학습 없음)")
    print(f"  경로 길이 {pi['dense_s'][-1]:.1f} m,  최소 회전반경 {1/np.abs(kap).max():.1f} m"
          f"  (Hockenheim 37.8 m)")
    print(f"  κ 가 학습 분포(1~99%) 밖인 비율: {100*np.mean((kap<lo)|(kap>hi)):.1f}%  -> 외삽 아님")
    print(f"  차선 폭 {LANE_W} m, 트럭 폭 {TRUCK_W} m  ->  좌우 여유 {MARGIN:.2f} m\n")

    print(f"{'':10s} {'완주':>5} {'거리':>8} {'평균|n|':>8} {'RMS|n|':>8} "
          f"{'최대|n|':>8} {'여유대비':>8} {'RMS α':>8} {'최대 α':>8} {'채터':>8}")
    print("-" * 90)
    rows = {}
    for f, l in [("hw_nom.npy", "nominal"), ("hw_res.npy", "residual")]:
        a = lap1(os.path.join(HW, f))
        s, n, al, de = a[:, 0], a[:, 1], a[:, 2], a[:, 5]
        d = s.max() - s.min()
        done = d > 850
        # alpha = 헤딩오차(yaw error, 경로 접선 대비 차체 방향). deg 로 보고한다.
        r = dict(dist=d, done=int(done), mean=np.abs(n).mean(),
                 rms=np.sqrt((n ** 2).mean()), max=np.abs(n).max(),
                 a_mean=np.rad2deg(np.abs(al).mean()),
                 a_rms=np.rad2deg(np.sqrt((al ** 2).mean())),
                 a_max=np.rad2deg(np.abs(al).max()),
                 chatter=np.sqrt(np.mean((np.diff(de) / 0.1) ** 2)))
        print(f"{l:10s} {'O' if done else 'X':>5} {d:7.0f}m {r['mean']:8.3f} "
              f"{r['rms']:8.3f} {r['max']:8.3f} {100*r['max']/MARGIN:7.0f}% "
              f"{r['a_rms']:7.2f}° {r['a_max']:7.2f}° {r['chatter']:8.4f}")
        rows[l] = r

    n_, r_ = rows["nominal"], rows["residual"]
    print(f"\n  평균 오차 {100*(n_['mean']-r_['mean'])/n_['mean']:.0f}% 감소, "
          f"최대 오차 {100*(n_['max']-r_['max'])/n_['max']:.0f}% 감소, "
          f"헤딩오차 RMS {100*(n_['a_rms']-r_['a_rms'])/n_['a_rms']:.0f}% 감소")
    print(f"  nominal 은 여유({MARGIN:.2f} m)를 {n_['max']/MARGIN:.2f}배 초과 -> 이탈")
    print(f"  residual 은 여유의 {100*r_['max']/MARGIN:.0f}% 만 사용 -> 완주")

    savemat(OUT, dict(
        lane_width=LANE_W, truck_width=TRUCK_W, margin=MARGIN,
        path_length=pi["dense_s"][-1], min_radius=1 / np.abs(kap).max(),
        kappa_outside_train_pct=100 * np.mean((kap < lo) | (kap > hi)),
        labels=np.array(["nominal", "residual"], dtype=object),
        completed=np.array([n_["done"], r_["done"]]),
        distance_m=np.array([n_["dist"], r_["dist"]]),
        mean_n=np.array([n_["mean"], r_["mean"]]),
        rms_n=np.array([n_["rms"], r_["rms"]]),
        max_n=np.array([n_["max"], r_["max"]]),
        mean_a=np.array([n_["a_mean"], r_["a_mean"]]),
        rms_a=np.array([n_["a_rms"], r_["a_rms"]]),
        max_a=np.array([n_["a_max"], r_["a_max"]]),
        chatter=np.array([n_["chatter"], r_["chatter"]]),
    ))
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
