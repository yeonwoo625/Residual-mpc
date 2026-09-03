#!/usr/bin/env python3
"""
참조 경로 vs 주행 궤적 — nominal MPC / residual MPC, 32 t 와 56 t.

Frenet 상태 (s, n) 을 전역 좌표로 되돌려 기준 경로 위에 겹쳐 그리기 위한 데이터.

    X = Xr(s) - sin(psi_r) * n
    Y = Yr(s) + cos(psi_r) * n
    (n > 0 = 진행방향 기준 왼쪽. reference_utils.frenet_to_cartesian 과 같은 규약)

**스케일 주의.** 트랙 한 바퀴가 2,540 m 인데 횡오차는 최대 1 m 남짓이라
전체 트랙을 그리면 세 선이 겹쳐 보인다. 그래서 가장 급한 코너 두 곳의
확대 창(zoom window)도 함께 저장한다. 확대 창 안에서는 nominal 과 residual 이
눈으로 구분된다.

주행: Q_n=1e-4, v=10, Hockenheim 건조.
      nominal  = results/traj_nom_{32,56}.npy
      residual = results/traj_ff_{32,56}.npy  (FF 잔차, scale 0.3, 6차원 모델)
      잔차 주행은 solve 109 ms 시절이라 실제 제어율이 6.5 Hz 다(nominal 은 10 Hz).

사용:  python3 results/track_compare.py
출력:  요약 + results/matlab/fig_track_compare.mat
"""
import os
import sys
import numpy as np
from scipy.io import savemat

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "mpc"))
from reference_utils import compute_path_spline                  # noqa: E402
from eval_b1 import _kappa_fn, evaluate, estimate_dt             # noqa: E402

WP     = "/home/vilab/CarMaker/mpc_host/hockenheim_waypoints_1lap.npy"
OUT    = os.path.join(HERE, "matlab", "fig_track_compare.mat")
MASSES = [32, 56]
HALF   = 55.0          # 확대 창 반폭 [m]


def lap1(path):
    a = np.load(path)
    w = np.where(np.diff(a[:, 0]) < -100)[0]
    return a[:w[0] + 1] if len(w) else a


def main():
    pi = compute_path_spline(np.load(WP))
    # compute_path_spline 은 x/y 를 스플라인 객체로 준다. 조밀 격자에서 평가한다.
    s_ref = pi["dense_s"]
    x_ref = pi["x_spline"](s_ref)
    y_ref = pi["y_spline"](s_ref)
    psi_ref, kap = pi["phi"], pi["kappa"]
    kf = _kappa_fn()

    def to_xy(s, n):
        sm = np.mod(s, s_ref[-1])
        xp = np.interp(sm, s_ref, x_ref)
        yp = np.interp(sm, s_ref, y_ref)
        ps = np.interp(sm, s_ref, psi_ref)
        return xp - np.sin(ps) * n, yp + np.cos(ps) * n

    # 확대 창: 곡률이 가장 큰 두 코너 (서로 200 m 이상 떨어진 곳)
    order = np.argsort(-np.abs(kap))
    picks = []
    for i in order:
        if all(abs(s_ref[i] - s_ref[j]) > 200 for j in picks):
            picks.append(i)
        if len(picks) == 2:
            break
    zoom_s = np.sort(s_ref[picks])
    zoom = np.array([[np.interp(z, s_ref, x_ref), np.interp(z, s_ref, y_ref)]
                     for z in zoom_s])

    print("참조 경로 vs 주행 궤적")
    print(f"  트랙 길이 {s_ref[-1]:.0f} m, 최소 회전반경 {1/np.abs(kap).max():.1f} m")
    print(f"  확대 창 중심: s = {zoom_s[0]:.0f} m, {zoom_s[1]:.0f} m  (반폭 {HALF:.0f} m)\n")

    # 궤적은 (적재, 제어기) 셀 배열로 저장한다. MATLAB 에서 traj_x{i,k} 로 꺼내
    # eval/sprintf 로 변수명을 조립할 필요가 없다.
    nM = len(MASSES)
    TX = np.empty((nM, 2), dtype=object); TY = np.empty((nM, 2), dtype=object)
    TS = np.empty((nM, 2), dtype=object); TN = np.empty((nM, 2), dtype=object)
    MET = np.zeros((nM, 2, 4))          # [rate_hz, corner_rms, mean|n|, max|n|]

    print(f"{'적재':>5}{'제어기':>14}{'제어율':>9}{'급코너|n|':>10}"
          f"{'평균|n|':>9}{'최대|n|':>9}")
    for i, m in enumerate(MASSES):
        for k, (tag, f) in enumerate((("nom", f"traj_nom_{m}.npy"),
                                      ("res", f"traj_ff_{m}.npy"))):
            fp = os.path.join(HERE, f)
            a = lap1(fp)
            TX[i, k], TY[i, k] = to_xy(a[:, 0], a[:, 1])
            TS[i, k], TN[i, k] = a[:, 0], a[:, 1]
            r = evaluate(fp, kf)
            MET[i, k] = [r["rate_hz"], r["corner_rms"], r["n_mean"], r["n_max"]]
            lbl = "Nominal MPC" if tag == "nom" else "Residual MPC"
            print(f"{m:4d}t{lbl:>14}{r['rate_hz']:8.1f}Hz{r['corner_rms']:10.3f}"
                  f"{r['n_mean']:9.3f}{r['n_max']:9.3f}")

    D = dict(ref_x=x_ref, ref_y=y_ref, ref_s=s_ref, ref_kappa=kap,
             s_total=s_ref[-1],
             zoom_xy=zoom, zoom_s=zoom_s, zoom_half=HALF,
             mass=np.array(MASSES),
             variant=np.array(["Nominal MPC", "Residual MPC"], dtype=object),
             metric=np.array(["rate_hz", "corner_n_rms", "mean_n", "max_n"],
                             dtype=object),
             traj_x=TX, traj_y=TY, traj_s=TS, traj_n=TN, met=MET)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    savemat(OUT, D)
    print(f"\n저장: {OUT}")
    print("주의: 전체 트랙 스케일에서는 1 m 편차가 보이지 않는다. 확대 창을 함께 본다.")


if __name__ == "__main__":
    main()
