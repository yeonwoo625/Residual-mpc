#!/usr/bin/env python3
"""
Tracking RMSE 표 — X / Y / Yaw, 조건별 nominal vs residual.

논문 표 형식(Trajectory | Method | X Error [m] | Y Error [m] | Yaw Error [rad])에
맞춘 요약이다. 지표 정의를 분명히 해 둔다.

  Y  (m)   횡오차 = Frenet n. 기준 경로에서 옆으로 벗어난 거리. RMSE.
  Yaw(rad) 헤딩오차 = Frenet alpha. 경로 접선 대비 차체 방향. RMSE.
  X  (m)   종방향 진행오차 = (실제 진행거리) - (목표속도 x 경과시간). RMSE.

**X 에 대한 주의.** 이 연구의 MPC 는 trajectory tracking 이 아니라 path
following 이다. 참조가 시간이 정해진 궤적이 아니라 경로이고, 비용의 종방향
목표가 `s_target = s + v_target*dt*(i+1)` 로 **현재 위치 기준 상대값**이다.
따라서 제어기에 절대 시간 기준이 없어 종방향 지연을 되돌릴 수단이 없고,
X 오차는 **설계상 누적된다**(랩이 길수록 커진다). 값이 Y 보다 두 자리 큰 것은
제어 실패가 아니라 이 구조 때문이다. 시간 궤적을 쫓는 논문의 X 와 직접
비교하면 안 된다.

대안 지표를 함께 저장한다.
  v_rmse (m/s)  속도추종 RMSE — 유계이며 종방향 성능을 왜곡 없이 나타낸다
  lap_time (s)  랩타임 — 종방향 지연의 최종 결과
  rate_hz       실제 제어 주기. 잔차 주행 일부는 solve 109 ms 라 6.5 Hz 로 돌았다
                (results/solvetime/ 참조). 표에 함께 싣는 이유는 제어율이 다르면
                공정한 비교가 아니기 때문이다. v=14 만 양쪽 10 Hz 다.

정지 출발 가속 구간은 어떤 제어기든 뒤처지므로 X 와 v_rmse 는 목표속도의 95% 에
처음 도달한 시점부터 잰다. Y 와 Yaw 는 전 구간이다.

사용:  python3 results/tracking_rmse.py
출력:  표 + results/matlab/fig_tracking_rmse.mat
"""
import os
import sys
import numpy as np
from scipy.io import savemat

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from eval_b1 import _kappa_fn, estimate_dt                    # noqa: E402

OUT = os.path.join(HERE, "matlab", "fig_tracking_rmse.mat")

# (표시 이름, nominal 파일, residual 파일, 목표속도)
SCEN = [
    ("Hockenheim  v=10",   "b1/v10_nom.npy",       "b1/v10_res.npy",       10.0),
    ("Hockenheim  v=11.5", "b1/v115_nom.npy",      "b1/v115_res.npy",      11.5),
    ("Hockenheim  v=12",   "b1/v12_nom.npy",       "b1/v12_res.npy",       12.0),
    ("Hockenheim  v=14",   "b1/v14_nom.npy",       "b1/v14_res.npy",       14.0),
    ("Highway No.1",       "highway/hw_nom.npy",   "highway/hw_res.npy",   12.0),
    ("Low friction  mu=0.3", "mu03/mu03_v9_nom.npy", "mu03/mu03_v9_res.npy", 9.0),
]
METHODS = ["Nominal MPC", "Residual MPC"]


def lap1(path):
    a = np.load(path)
    w = np.where(np.diff(a[:, 0]) < -100)[0]
    return a[:w[0] + 1] if len(w) else a


def metrics(path, v_target):
    a = lap1(path)
    s, n, alpha, v = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    dt = estimate_dt(s, v)
    i = int(np.argmax(v >= 0.95 * v_target))
    t = np.arange(len(s) - i) * dt
    xe = (s[i:] - s[i]) - v_target * t
    return dict(
        x     = np.sqrt((xe ** 2).mean()),
        y     = np.sqrt((n ** 2).mean()),
        yaw   = np.sqrt((alpha ** 2).mean()),          # rad
        v_rms = np.sqrt(((v[i:] - v_target) ** 2).mean()),
        lap   = (len(a) - 1) * dt,
        rate  = 1.0 / dt,
        dist  = s.max() - s.min(),
    )


def main():
    X = np.zeros((len(SCEN), 2)); Y = np.zeros_like(X); YAW = np.zeros_like(X)
    VR = np.zeros_like(X); LAP = np.zeros_like(X); RATE = np.zeros_like(X)

    print("Tracking RMSE Comparison\n")
    print(f"{'Scenario':<22}{'Method':<14}{'X (m)':>9}{'Y (m)':>9}"
          f"{'Yaw (rad)':>11}{'v (m/s)':>9}{'lap (s)':>9}{'rate':>8}")
    print("-" * 91)
    for i, (name, fn, fr, vt) in enumerate(SCEN):
        for j, f in enumerate((fn, fr)):
            m = metrics(os.path.join(HERE, f), vt)
            X[i, j], Y[i, j], YAW[i, j] = m["x"], m["y"], m["yaw"]
            VR[i, j], LAP[i, j], RATE[i, j] = m["v_rms"], m["lap"], m["rate"]
            lbl = name if j == 0 else ""
            print(f"{lbl:<22}{METHODS[j]:<14}{m['x']:9.2f}{m['y']:9.3f}"
                  f"{m['yaw']:11.4f}{m['v_rms']:9.3f}{m['lap']:9.1f}{m['rate']:7.1f}Hz")
        print()

    print("주의")
    print("  · X 는 path following 구조상 누적된다(절대 시간 기준 없음). Y 와 직접")
    print("    비교하거나 trajectory tracking 논문의 X 와 나란히 놓으면 안 된다.")
    print("    종방향 성능은 v(속도추종 RMSE)와 lap 으로 읽는 편이 정확하다.")
    bad = [(SCEN[i][0], RATE[i, 1]) for i in range(len(SCEN)) if RATE[i, 1] < 9.0]
    if bad:
        print("  · 제어율이 10 Hz 가 아닌 잔차 주행 (solve 109 ms, 2026-09-02 수정 전):")
        for nm, r in bad:
            print(f"      {nm:<24}{r:.1f} Hz")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    savemat(OUT, dict(
        scenario = np.array([s[0] for s in SCEN], dtype=object),
        method   = np.array(METHODS, dtype=object),
        v_target = np.array([s[3] for s in SCEN]),
        x_rmse   = X,        # (조건, 방법) 방법 0=nominal 1=residual
        y_rmse   = Y,
        yaw_rmse = YAW,
        v_rmse   = VR,
        lap_time = LAP,
        rate_hz  = RATE,
    ))
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
