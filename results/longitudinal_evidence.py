#!/usr/bin/env python3
"""
종방향 잔차를 쓰지 않은 근거 — 세 갈래 증거.

배포 설정(mpc_solver_residual.B_MATRIX)은 잔차의 횡방향 채널(Δn, Δα)만 쓰고
종방향(Δs, Δv)은 버린다. 그 근거를 측정으로 제시한다.

  (a) 과도구간의 존재
      스로틀이 D=1.00 으로 고정인데 가속도가 반복적으로 붕괴한다. 정지 출발
      가속에서 8회 검출. 차량은 토크컨버터 자동변속기(Auto_Conv, 클러치 Closed)
      이므로 클러치 단절이 아니라 기어비 전환에 따른 구동력 교란이다.

  (b) 신경망이 실제로 맞히는 정도
      검증 분할에서 예측 대 실제 잔차. 횡방향은 상관 0.98~0.99, 종방향은 0.28.
      Δv 는 R²=0.074 로 잔차의 7% 만 설명한다.

  (c) 어디가 문제인가
      과도구간(주행의 8%)의 설명불가가 59.3%, 정속구간(88%)은 4.4%.
      즉 종방향이 원리적으로 학습 불가한 것이 아니라 과도구간이 문제다.
      다만 정속구간의 잔차는 2초 예측구간에서 0.15 m/s 로 보정 실익이 작다.

**주의 — 주장하지 않는 것.** "종방향 잔차를 켜면 발산한다"는 코드 주석의 추측이며
검증 주행 기록이 없다. 본 스크립트는 학습 가능성과 보정 실익만 근거로 삼는다.

**한계.** 정속 보정량 0.15 m/s 는 학습 데이터 속도(약 12 m/s)에서 잰 값이다.
종방향 모델은 저속(0~8 m/s) 데이터로 식별돼 그 부근에서만 오차가 상쇄되므로,
20 m/s 에서는 잔차가 더 클 수 있다(미측정).

사용:  python3 results/longitudinal_evidence.py
출력:  표 3개 + results/matlab/fig_longitudinal_evidence.mat
"""
import os
import sys
import numpy as np
import torch
from scipy.io import savemat

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "mpc"))
from residual_model_wrapper import load_normalized_model            # noqa: E402
from channel_learnability import knn_floor                          # noqa: E402

DATA   = "/home/vilab/CarMaker/mpc_host/hockenheim_mass.npz"
LAUNCH = os.path.join(HERE, "b1", "v12_nom.npy")   # 정지 출발이 있는 주행
MODEL  = os.path.join(ROOT, "mpc", "residual_model_nomass_s0.pt")
OUT    = os.path.join(HERE, "matlab", "fig_longitudinal_evidence.mat")
DT     = 0.1
CH     = ["ds", "dn", "dalpha", "dv"]
A_STEADY, A_TRANS = 0.1, 0.2      # 정속 / 과도 판정 [m/s^2]


def block_accel(X, mass_col=6):
    """적재 블록(=주행) 안에서만 가속도를 계산한다. 경계는 NaN."""
    m = np.round(X[:, mass_col] / 1000).astype(int)
    b = list(np.where(np.diff(m) != 0)[0] + 1)
    acc = np.full(len(X), np.nan)
    for lo, hi in zip([0] + b, b + [len(m)]):
        acc[lo:hi] = np.gradient(X[lo:hi, 2], DT)
    return acc


def find_shifts(v, D, acc, thr=0.35, win=8):
    """스로틀이 큰데 가속도가 국소 중앙값의 thr 배 아래로 떨어지는 지점."""
    idx = []
    for k in range(win, len(acc)):
        if D[k] < 0.6:
            continue
        med = np.median(acc[k - win:k])
        if med > 0.3 and acc[k] < thr * med and (not idx or k - idx[-1] > 3):
            idx.append(k)
    return np.array(idx, dtype=int)


def main():
    d = np.load(DATA)
    X, y = d["X"], d["y"]
    acc = block_accel(X)

    # ---------- (a) 과도구간의 존재 ----------
    a = np.load(LAUNCH)
    w = np.where(np.diff(a[:, 0]) < -100)[0]
    if len(w):
        a = a[:w[0] + 1]
    lv, lD = a[:, 3], a[:, 4]
    lacc = np.gradient(lv, DT)
    i0 = int(np.argmax(lv > 0.5))
    i1 = int(np.argmax(lv > 11.5)) if (lv > 11.5).any() else len(lv)
    lt = (np.arange(i0, i1) - i0) * DT
    lv, lD, lacc = lv[i0:i1], lD[i0:i1], lacc[i0:i1]
    sh = find_shifts(lv, lD, lacc)

    print("=" * 72)
    print("(a) 과도구간 — 스로틀 고정인데 가속도가 붕괴하는 지점")
    print("=" * 72)
    print(f"{'t(s)':>7}{'v(m/s)':>9}{'D':>7}{'가속도':>10}{'직전 중앙값':>12}")
    for k in sh:
        print(f"{lt[k]:7.1f}{lv[k]:9.2f}{lD[k]:7.2f}{lacc[k]:10.3f}"
              f"{np.median(lacc[k-8:k]):12.3f}")
    print(f"\n  {len(sh)}회 검출. 전부 D >= 0.6 (스로틀 유지) 상태다.")
    print("  차량은 Auto_Conv(토크컨버터 자동) + Clutch Closed 이므로")
    print("  클러치 단절이 아니라 기어비 전환에 따른 구동력 교란이다.")

    # ---------- (b) 신경망이 맞히는 정도 ----------
    nval = int(len(X) * 0.2)
    idx = np.random.default_rng(42).permutation(len(X))
    Xv, yv, av = X[idx][:nval], y[idx][:nval], acc[idx][:nval]
    model = load_normalized_model(MODEL, device="cpu")
    with torch.no_grad():
        pv = model(torch.from_numpy(Xv[:, :6].astype(np.float32))).numpy()

    FIT = np.zeros((4, 4))      # [corr, R2, RMS감소%, 실제RMS]
    print("\n" + "=" * 72)
    print(f"(b) 신경망 예측 대 실제 잔차 (검증 분할 {nval} 샘플, 학습 미사용)")
    print("=" * 72)
    print(f"{'채널':>10}{'상관계수':>10}{'R2':>9}{'RMS 감소':>10}{'실제 RMS':>11}")
    for j in range(4):
        t, q = yv[:, j], pv[:, j]
        r = np.corrcoef(t, q)[0, 1]
        r2 = 1 - np.sum((t - q) ** 2) / np.sum((t - t.mean()) ** 2)
        rms = np.sqrt((t ** 2).mean())
        red = 100 * (1 - np.sqrt(((t - q) ** 2).mean()) / rms)
        FIT[j] = [r, r2, red, rms]
        print(f"{CH[j]:>10}{r:10.3f}{r2:9.3f}{red:9.1f}%{rms:11.5f}")
    print("\n  횡(dn, dalpha) 상관 0.98~0.99  vs  종(dv) 0.28.")
    print("  Δv 는 R2=0.074 로 잔차의 7% 만 설명한다.")

    # ---------- (c) 정속 / 과도 분해 ----------
    Z = (X[:, :6] - X[:, :6].mean(0)) / (X[:, :6].std(0) + 1e-9)
    ok = ~np.isnan(acc)
    regimes = [("all", ok),
               ("steady |a|<%.1f" % A_STEADY, ok & (np.abs(acc) < A_STEADY)),
               ("transient |a|>=%.1f" % A_TRANS, ok & (np.abs(acc) >= A_TRANS))]
    # 필드명 floor 는 MATLAB 내장 함수와 겹치므로 unexpl 로 저장한다
    FLOOR = np.zeros((len(regimes), 2))     # [dv, dn]
    FRAC = np.zeros(len(regimes))
    print("\n" + "=" * 72)
    print("(c) 정속 / 과도 분해 — 설명 불가 비율 (k-NN 노이즈 실링)")
    print("=" * 72)
    print(f"{'구간':>24}{'샘플':>8}{'비중':>7}{'Δv':>9}{'Δn':>9}")
    for i, (lbl, sel) in enumerate(regimes):
        f = knn_floor(Z[sel], y[sel])
        FLOOR[i] = [f[3], f[1]]
        FRAC[i] = sel.mean()
        print(f"{lbl:>24}{sel.sum():8d}{100*sel.mean():6.0f}%"
              f"{100*f[3]:8.1f}%{100*f[1]:8.1f}%")

    st = ok & (np.abs(acc) < A_STEADY)
    bias = y[st, 3].mean()
    print(f"\n  정속 구간 Δv 평균 = {bias:+.5f} m/s/step"
          f"  ->  2초(20스텝) 누적 {bias*20:+.3f} m/s")
    print("  즉 보정이 필요한 구간에서는 학습이 불가능하고,")
    print("  학습이 가능한 구간에서는 보정량이 작다.")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    savemat(OUT, dict(
        # (a)
        t=lt, v_launch=lv, D_launch=lD, acc_launch=lacc,
        shift_idx=sh + 1, shift_v=lv[sh], shift_acc=lacc[sh],
        # (b)
        y_true=yv, y_pred=pv, fit=FIT,
        fit_col=np.array(["corr", "R2", "rms_reduction_pct", "true_rms"],
                         dtype=object),
        channel=np.array(CH, dtype=object), n_val=nval,
        # (c)
        regime=np.array([r[0] for r in regimes], dtype=object),
        unexpl=FLOOR, unexpl_col=np.array(["dv", "dn"], dtype=object),
        regime_frac=FRAC, steady_bias=bias, horizon_steps=20,
        a_steady=A_STEADY, a_trans=A_TRANS,
    ))
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
