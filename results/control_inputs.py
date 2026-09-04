#!/usr/bin/env python3
"""
제어 입출력 비교 — nominal vs residual, 세 조건.

MPC 가 차량에 보내는 것은 스로틀 D 와 조향각 delta 다(속도가 아니다). 상태
[s, n, alpha, v, D, delta] 중 D 와 delta 가 실제 명령이고, 그 변화율
[derD, derDelta] 가 최적화 변수다. 그래서 delta 의 시간 미분(조향각속도)까지
함께 본다 - 제약 delta<=30deg, ddelta 는 DDELTA_MAX 로 건다.

차량 응답(속도 v, 가속도 a = dv/dt)도 함께 저장한다. 속도는 명령이 아니라
스로틀의 결과이므로, "명령(D, delta) -> 응답(v, a)" 을 나란히 봐야 제어기가
무엇을 했고 차량이 어떻게 반응했는지 읽힌다. 가속도는 제어 주기로 나눈 값이라
제어율을 데이터에서 역산해 쓴다.

  세 조건
    저마찰      mu=0.3, v=9,  Hockenheim (완주한 유일한 저마찰 쌍)
    고속        v=20, Highway L1 (초기속도 72 km/h, 주행의 84% 가 v>=15)
    기준        v=12, Hockenheim

**제어율 주의.** 잔차 주행 중 저마찰(6.4 Hz)과 v=12(6.5 Hz)는 solve 가 109 ms 이던
시절이라 10 Hz 를 못 지켰다. 고속 조건만 양쪽 10 Hz 다. 조향각속도는 dt 로 나누므로
제어율을 데이터에서 역산해 쓴다(DT=0.1 고정 시 6.5 Hz 주행이 1.55배 과대평가된다).

가로축은 경로거리 s 와 시간 t 를 모두 저장한다(MATLAB 쪽에서 전환).
  s  같은 지점을 나란히 비교할 때. 두 주행의 속도가 달라도 위치가 맞는다.
  t  속도가 어떻게 올라가고 유지되는지 볼 때. 제어율이 달라도 초 단위는 같으나,
     같은 t 가 같은 지점을 뜻하지는 않는다.

사용:  python3 results/control_inputs.py
출력:  표 + results/matlab/fig_control_inputs.mat
"""
import os
import sys
import numpy as np
from scipy.io import savemat

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from eval_b1 import estimate_dt                                   # noqa: E402

HOST = "/home/vilab/CarMaker/mpc_host"
OUT  = os.path.join(HERE, "matlab", "fig_control_inputs.mat")
R2D  = 180.0 / np.pi

CASES = [
    ("Low friction  mu=0.3, v=9",
     os.path.join(HERE, "mu03", "mu03_v9_nom.npy"),
     os.path.join(HERE, "mu03", "mu03_v9_res.npy")),
    ("Highway L1  v=20",
     os.path.join(HOST, "hw_nom_v20i.npy"),
     os.path.join(HOST, "hw_res_v20i.npy")),
    ("Hockenheim  v=12",
     os.path.join(HERE, "b1", "v12_nom.npy"),
     os.path.join(HERE, "b1", "v12_res.npy")),
]
VARIANT = ["Nominal MPC", "Residual MPC"]
DELTA_MAX_DEG = 30.0        # bicycle_model.delta_max


def lap1(path):
    a = np.load(path)
    w = np.where(np.diff(a[:, 0]) < -100)[0]
    return a[:w[0] + 1] if len(w) else a


def main():
    nC = len(CASES)
    SS = np.empty((nC, 2), dtype=object)   # 경로거리
    DD = np.empty((nC, 2), dtype=object)   # 스로틀
    DE = np.empty((nC, 2), dtype=object)   # 조향각 [deg]
    DR = np.empty((nC, 2), dtype=object)   # 조향각속도 [deg/s]
    VV = np.empty((nC, 2), dtype=object)   # 속도
    AA = np.empty((nC, 2), dtype=object)   # 가속도 [m/s^2]
    TT = np.empty((nC, 2), dtype=object)   # 시간 [s]. 제어율이 달라 스텝수x dt 로 만든다
    # [rate_hz, D평균, 풀스로틀%, 제동%, |delta|최대, delta_rate RMS,
    #  delta_rate 최대, v평균, v최대, a평균, |a|최대]
    MET = np.zeros((nC, 2, 11))

    print("제어 입출력 — MPC 가 차량에 보내는 값\n")
    print(f"{'조건':>26}{'제어기':>14}{'Hz':>6}{'D평균':>8}{'풀':>6}{'제동':>6}"
          f"{'|δ|최대':>9}{'δ̇ RMS':>9}{'δ̇ 최대':>9}")
    for i, (lbl, fn, fr) in enumerate(CASES):
        for k, f in enumerate((fn, fr)):
            a = lap1(f)
            s, v, D, de = a[:, 0], a[:, 3], a[:, 4], a[:, 5]
            dt = estimate_dt(s, v)
            rate = np.gradient(de, dt) * R2D
            acc = np.gradient(v, dt)
            TT[i, k] = np.arange(len(s)) * dt
            SS[i, k], DD[i, k] = s, D
            DE[i, k], DR[i, k], VV[i, k], AA[i, k] = de * R2D, rate, v, acc
            MET[i, k] = [1 / dt, D.mean(), 100 * np.mean(D > 0.99),
                         100 * np.mean(D < 0), np.abs(de * R2D).max(),
                         np.sqrt((rate ** 2).mean()), np.abs(rate).max(),
                         v.mean(), v.max(), acc.mean(), np.abs(acc).max()]
            nm = lbl if k == 0 else ""
            print(f"{nm:>26}{VARIANT[k]:>14}{MET[i,k,0]:6.1f}{MET[i,k,1]:8.3f}"
                  f"{MET[i,k,2]:5.0f}%{MET[i,k,3]:5.0f}%{MET[i,k,4]:9.1f}"
                  f"{MET[i,k,5]:9.2f}{MET[i,k,6]:9.1f}")
        print()

    print(f"{'조건':>26}{'제어기':>14}{'평균v':>8}{'최대v':>8}{'평균a':>9}{'|a|최대':>9}")
    for i, (lbl, _, _) in enumerate(CASES):
        for k in range(2):
            nm = lbl if k == 0 else ""
            print(f"{nm:>26}{VARIANT[k]:>14}{MET[i,k,7]:8.2f}{MET[i,k,8]:8.2f}"
                  f"{MET[i,k,9]:+9.3f}{MET[i,k,10]:9.2f}")
        print()

    print("주의")
    print("  · 조향각속도는 제어 주기로 나눈 값이다. 잔차 주행 중 저마찰(6.4 Hz)과")
    print("    v=12(6.5 Hz)는 솔버 수정 전이라 10 Hz 를 못 지켰다. 고속 조건만")
    print("    양쪽 10 Hz 로 제어율이 맞는 비교다.")
    print("  · 제동은 D < 0 구간이다. 세 조건 모두 1% 이하로 거의 쓰이지 않는다.")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    savemat(OUT, dict(
        # 필드명 case 는 MATLAB 예약어(switch/case)라 case_ 로 둔다
        case_=np.array([c[0] for c in CASES], dtype=object),
        variant=np.array(VARIANT, dtype=object),
        metric=np.array(["rate_hz", "D_mean", "full_throttle_pct", "braking_pct",
                         "max_abs_delta_deg", "delta_rate_rms_dps",
                         "max_delta_rate_dps", "v_mean", "v_max",
                         "a_mean", "max_abs_a"], dtype=object),
        met=MET, delta_max_deg=DELTA_MAX_DEG,
        s=SS, t=TT, throttle=DD, delta_deg=DE, delta_rate_dps=DR,
        speed=VV, accel=AA,
    ))
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
