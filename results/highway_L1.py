#!/usr/bin/env python3
"""
Highway No.1 (좌측 1 m 이동 경로), v=12 — nominal vs residual.

배경: 원래 기준 경로는 IPGDriver 주행선(CornerCutCoef=0.85)이라 우측에 치우친
구간이 있었다. 갓길 폭이 좌우 비대칭이고(왼쪽 0.75~4.50 m, 오른쪽 0.21~1.99 m)
구간마다 변해서, nominal 이 s=625 m 에서 횡편차 0.46 m 만으로 우측 타이어가
도로를 벗어나 SIM_ABORT 가 났다(2026-09-03 재현 확인).

경로를 좌측으로 1 m 평행이동해(scripts/shift_path.py) 우측 여유를 확보한 뒤
양쪽을 다시 주행했다. **두 주행 모두 10 Hz 로 동작한다** — 솔버 수정(SQP_ITER=10)
이후라, 잔차가 6.5~6.9 Hz 로 돌던 기존 주행과 달리 제어율이 맞는 비교다.

  경로   highway_waypoints_L1.npy   932 m, 최소반경 90.2 m
  주행   v=12, 48 t, Q_n=1e-4, 잔차 scale 0.3, 6차원 모델(재학습 없음)
  결과   양쪽 완주 926 m

**평가 구간.** L1 경로는 차량 초기 위치에서 1 m 떨어져 있어 출발 직후 |n| 이
1.0 m 로 시작한다(제어 성능이 아니라 초기 오프셋). s < 30 m 를 빼고 집계한다.
과도구간은 s ~ 16 m 에서 |n| < 0.15 로 가라앉는다.

**급코너 정의.** 이 도로는 |kappa| 최대가 0.0111 (R=90 m) 로 Hockenheim 기준
0.02 를 한 번도 넘지 않는다. 저장소 관례(corner_mask.m)대로 곡률 상위 20%
(|kappa| >= 0.0081, R <= 123 m)를 급코너로 본다.

사용:  python3 results/highway_L1.py
출력:  표 + results/matlab/fig_highway_L1.mat
"""
import os
import sys
import numpy as np
from scipy.io import savemat
from scipy.interpolate import make_interp_spline

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "mpc"))
from reference_utils import compute_path_spline                  # noqa: E402
from eval_b1 import estimate_dt                                  # noqa: E402

HOST   = "/home/vilab/CarMaker/mpc_host"
WP     = os.path.join(HOST, "highway_waypoints_L1.npy")
RUNS   = [("Nominal MPC", "hw_nom_L1.npy"), ("Residual MPC", "hw_res_L1.npy")]
OUT    = os.path.join(HERE, "matlab", "fig_highway_L1.mat")
S_MIN  = 30.0        # 출발 오프셋 제외
CORNER_PCT = 80      # 곡률 상위 20% 를 급코너로
REF_STEP = 5         # 기준 경로 솎기 (0.1 m -> 0.5 m 간격)


def main():
    pi = compute_path_spline(np.load(WP))
    s_ref = pi["dense_s"]
    x_ref = pi["x_spline"](s_ref)
    y_ref = pi["y_spline"](s_ref)
    psi   = np.unwrap(pi["phi"])          # 감긴 채 보간하면 좌표 변환이 튄다
    kap   = pi["kappa"]
    kthr  = np.percentile(np.abs(kap), CORNER_PCT)
    kf    = make_interp_spline(s_ref, kap, k=3)

    print(f"Highway L1 경로: {s_ref[-1]:.0f} m, 최소반경 {1/np.abs(kap).max():.1f} m")
    print(f"급코너 기준 |kappa| >= {kthr:.5f} (R <= {1/kthr:.0f} m, 상위 {100-CORNER_PCT}%)")
    print(f"평가 구간 s >= {S_MIN:.0f} m (출발 오프셋 제외)\n")

    TS, TN, TA, TV, TX, TY = ({} for _ in range(6))
    M = np.zeros((len(RUNS), 8))   # y_rmse y_mean y_max yaw_rmse yaw_max corner rate dist
    for i, (lbl, f) in enumerate(RUNS):
        a = np.load(os.path.join(HOST, f))
        s, n, al, v = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
        dt = estimate_dt(s, v)
        sm = np.mod(s, s_ref[-1])
        TX[i] = np.interp(sm, s_ref, x_ref) - np.sin(np.interp(sm, s_ref, psi)) * n
        TY[i] = np.interp(sm, s_ref, y_ref) + np.cos(np.interp(sm, s_ref, psi)) * n
        TS[i], TN[i], TA[i], TV[i] = s, n, al, v

        w = s >= S_MIN
        c = w & (np.abs(kf(s)) >= kthr)
        M[i] = [np.sqrt((n[w] ** 2).mean()), np.abs(n[w]).mean(), np.abs(n[w]).max(),
                np.rad2deg(np.sqrt((al[w] ** 2).mean())), np.rad2deg(np.abs(al[w]).max()),
                np.sqrt((n[c] ** 2).mean()), 1.0 / dt, s.max() - s.min()]

    hdr = ["Y RMSE [m]", "mean|n| [m]", "max|n| [m]", "Yaw RMSE [deg]",
           "max|a| [deg]", "corner Y RMSE [m]", "rate [Hz]", "distance [m]"]
    print(f"{'':>14}" + "".join(f"{h:>18}" for h in hdr[:3]))
    for i, (lbl, _) in enumerate(RUNS):
        print(f"{lbl:>14}" + "".join(f"{M[i,j]:18.3f}" for j in range(3)))
    print(f"{'개선':>14}" + "".join(f"{100*(M[0,j]-M[1,j])/M[0,j]:17.1f}%" for j in range(3)))
    print()
    print(f"{'':>14}" + "".join(f"{h:>18}" for h in hdr[3:6]))
    for i, (lbl, _) in enumerate(RUNS):
        print(f"{lbl:>14}" + "".join(f"{M[i,j]:18.3f}" for j in range(3, 6)))
    print(f"{'개선':>14}" + "".join(f"{100*(M[0,j]-M[1,j])/M[0,j]:17.1f}%" for j in range(3, 6)))
    print()
    for i, (lbl, _) in enumerate(RUNS):
        print(f"{lbl:>14}  제어율 {M[i,6]:.1f} Hz,  주행거리 {M[i,7]:.0f} m")

    sl = slice(None, None, REF_STEP)
    D = dict(
        ref_x=x_ref[sl], ref_y=y_ref[sl], ref_s=s_ref[sl],
        ref_psi=psi[sl], ref_kappa=kap[sl],
        s_total=s_ref[-1], kappa_thr=kthr, s_min=S_MIN,
        variant=np.array([r[0] for r in RUNS], dtype=object),
        metric=np.array(hdr, dtype=object),
        met=M,
    )
    for key, src in (("traj_s", TS), ("traj_n", TN), ("traj_a", TA),
                     ("traj_v", TV), ("traj_x", TX), ("traj_y", TY)):
        arr = np.empty(len(RUNS), dtype=object)
        for i in range(len(RUNS)):
            arr[i] = src[i]
        D[key] = arr

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    savemat(OUT, D)
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
