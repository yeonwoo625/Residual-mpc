#!/usr/bin/env python3
"""
종방향 잔차를 쓰지 않은 근거 — 기어 변속 때문에 학습이 불가능하다.

배포 설정(mpc_solver_residual.B_MATRIX)은 잔차의 횡방향 채널(Δn, Δα)만 쓰고
종방향(Δs, Δv)은 버린다. 코드 주석에는 "Δv 가 제일 크고 거칠어 발산 의심"이라고만
적혀 있어 근거가 약했다. 데이터로 원인을 짚는다.

세 갈래 증거.

  ① 변속 흔적 — 스로틀이 D=1.00 으로 고정인데 가속도가 반복적으로 붕괴한다.
     정지 출발 가속 중 v ~ 3.6 / 4.3 / 5.2 / 6.4 m/s 에서 가속도가 1.1 m/s^2 에서
     0.05~0.12 로 떨어졌다가 회복한다. 토크 단절 = 변속이다.

  ② 잔차가 그 구간에 몰린다 — |Δv| 상위 1% 샘플의 25~30배가 v = 3~6 m/s 에
     집중된다. 순항 구간(11~12 m/s, 전체의 90%)에는 상위 1% 가 하나도 없다.

  ③ 같은 입력에 다른 정답 — (v, D) 를 좁게 묶어도 Δv 는 크게 흩어진다.
     v 3~4 m/s, D>0.9 에서 Δv 가 -0.058 ~ +0.271 (표준편차 0.101) 인데
     같은 구간 Δn 의 표준편차는 0.0002 다. 500배 차이다.
     변속기 내부 상태(현재 단수, 변속 중 여부)가 MLP 입력에 없으므로
     모델이 두 경우를 구분할 방법이 없다.

이는 results/channel_learnability.py 의 노이즈 실링과 일치한다 — 같은 상태에서
관측된 잔차 산포가 Δn 은 1.6%, Δv 는 50.1% 로 31배다. 속도를 11.5~12.5 로
한정하면 Δv 의 50.1% 가 12.8% 로 떨어진다(변속이 없는 구간이기 때문).

**검증 가능한 예측:** 기어 단수를 MLP 입력에 넣으면 Δv 의 노이즈 실링이 내려가야
한다. 현재 /hmg_vehicle_state 에 기어 필드가 없어(x,y,yaw,v,vx,vy,omega,throttle,
steer) 미실시다. CMRosIF 브리지 수정 + 데이터 재수집이 필요하다.

사용:  python3 results/longitudinal_limit.py
출력:  표 3개 + results/matlab/fig_longitudinal_limit.mat
"""
import os
import sys
import numpy as np
from scipy.io import savemat

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from channel_learnability import knn_floor                     # noqa: E402

DATA  = "/home/vilab/CarMaker/mpc_host/hockenheim_mass.npz"
LAUNCH = os.path.join(HERE, "b1", "v12_nom.npy")   # 정지 출발이 있는 주행
OUT   = os.path.join(HERE, "matlab", "fig_longitudinal_limit.mat")
DT    = 0.1
BINS  = [(3.0, 4.0), (5.0, 6.0), (7.0, 8.0), (9.0, 10.0), (11.5, 12.0)]


def launch_segment():
    """정지 출발 가속 구간의 시간, 속도, 스로틀, 가속도."""
    a = np.load(LAUNCH)
    v, D = a[:, 3], a[:, 4]
    acc = np.gradient(v, DT)
    i0 = int(np.argmax(v > 0.5))
    i1 = int(np.argmax(v > 11.5)) if (v > 11.5).any() else len(v)
    sl = slice(i0, i1)
    t = (np.arange(i0, i1) - i0) * DT
    return t, v[sl], D[sl], acc[sl]


def find_shifts(v, D, acc, thr=0.35, win=8):
    """스로틀이 큰데 가속도가 국소 중앙값의 thr 배 아래로 떨어지는 지점."""
    idx = []
    for k in range(win, len(acc)):
        if D[k] < 0.6:
            continue
        med = np.median(acc[k - win:k])
        if med > 0.3 and acc[k] < thr * med:
            if not idx or k - idx[-1] > 3:      # 인접 중복 제거
                idx.append(k)
    return np.array(idx, dtype=int)


def main():
    d = np.load(DATA)
    X, y = d["X"], d["y"]
    v, D = X[:, 2], X[:, 3]
    dv, dn = y[:, 3], y[:, 1]

    # ---------- ① 변속 흔적 ----------
    t, lv, lD, lacc = launch_segment()
    sh = find_shifts(lv, lD, lacc)
    print("=" * 70)
    print("① 변속 흔적 — 스로틀 고정인데 가속도가 붕괴하는 지점")
    print("=" * 70)
    print(f"{'t(s)':>7}{'v(m/s)':>9}{'D':>7}{'가속도':>10}{'직전 중앙값':>12}")
    for k in sh:
        print(f"{t[k]:7.1f}{lv[k]:9.2f}{lD[k]:7.2f}{lacc[k]:10.3f}"
              f"{np.median(lacc[k-8:k]):12.3f}")
    print(f"\n  검출 {len(sh)}회. 전부 D >= 0.6 (스로틀 유지) 상태다.")

    # ---------- ② 잔차가 몰리는 곳 ----------
    big = np.abs(dv) > np.percentile(np.abs(dv), 99)
    edges = np.arange(0, 13, 1.0)
    ctr, conc, cnt = [], [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (v >= lo) & (v < hi)
        if m.sum() < 20:
            continue
        ctr.append((lo + hi) / 2)
        cnt.append(int(m.sum()))
        conc.append(((m & big).sum() / max(big.sum(), 1)) / (m.sum() / len(v)))
    print("\n" + "=" * 70)
    print("② |Δv| 상위 1% 가 어느 속도에 몰리나 (집중도 1 = 고르게 분포)")
    print("=" * 70)
    print(f"{'속도':>10}{'샘플':>8}{'집중도':>9}")
    for c, n_, k in zip(ctr, cnt, conc):
        print(f"{c:9.1f}{n_:8d}{k:8.1f}x")

    # ---------- ③ 같은 입력, 다른 정답 ----------
    print("\n" + "=" * 70)
    print("③ (v, D) 를 좁게 묶었을 때의 잔차 산포 — 입력이 같아도 정답이 다른가")
    print("=" * 70)
    print(f"{'속도 구간':>14}{'샘플':>6}{'Δv 표준편차':>13}{'Δn 표준편차':>13}{'배수':>8}")
    B = np.zeros((len(BINS), 4))       # [v_center, n, std_dv, std_dn]
    for bi, (lo, hi) in enumerate(BINS):
        m = (v >= lo) & (v < hi) & (D > 0.9)
        if m.sum() < 15:
            m = (v >= lo) & (v < hi)
        B[bi] = [(lo + hi) / 2, m.sum(), dv[m].std(), dn[m].std()]
        print(f"  {lo:5.1f}~{hi:5.1f}{int(m.sum()):6d}{dv[m].std():12.4f}"
              f"{dn[m].std():13.4f}{dv[m].std()/max(dn[m].std(),1e-9):7.0f}x")

    # ---------- 노이즈 실링 ----------
    Z = (X[:, :6] - X[:, :6].mean(0)) / (X[:, :6].std(0) + 1e-9)
    f_all = knn_floor(Z, y)
    m_cr = (v >= 11.5) & (v < 12.5)
    f_cr = knn_floor(Z[m_cr], y[m_cr])
    print("\n" + "=" * 70)
    print("설명 불가 비율 (k-NN 노이즈 실링) — 낮을수록 학습 가능")
    print("=" * 70)
    print(f"{'':>16}{'Δn (횡)':>10}{'Δv (종)':>10}")
    print(f"{'전 구간':>16}{100*f_all[1]:9.1f}%{100*f_all[3]:9.1f}%")
    print(f"{'순항 11.5~12.5':>16}{100*f_cr[1]:9.1f}%{100*f_cr[3]:9.1f}%")
    print("\n  순항 구간(변속 없음)만 보면 Δv 의 설명불가가 50.1% -> 12.8% 로 떨어진다.")
    print("  종방향이 못 배우는 것은 모델 탓이 아니라 변속 정보가 입력에 없어서다.")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    savemat(OUT, dict(
        t=t, v_launch=lv, D_launch=lD, acc_launch=lacc,
        shift_idx=sh + 1,                     # MATLAB 1-based
        shift_v=lv[sh], shift_acc=lacc[sh],
        bin_center=np.array(ctr), bin_count=np.array(cnt),
        concentration=np.array(conc),
        spread=B,
        spread_dim=np.array(["v_center", "n", "std_dv", "std_dn"], dtype=object),
        floor_all=f_all, floor_cruise=f_cr,
        channel=np.array(["ds", "dn", "dalpha", "dv"], dtype=object),
    ))
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
