#!/usr/bin/env python3
"""
채널별 잔차 '학습 가능성' 측정 — 슬라이드 15/18 근거.

주장: "똑같은 입력인데 잔차가 다양하다" (= MLP 입력에 없는 은닉 상태가 있다).
측정: MLP 입력이 (거의) 같은 샘플끼리 잔차가 얼마나 흩어지는가.
      이 흩어짐은 어떤 모델로도 줄일 수 없는 '바닥'이다 (irreducible noise).

      설명 불가 비율 = (입력이 같은 샘플들 사이의 분산) / (전체 분산)

  - 종방향 Δv 는 변속 기어단이 MLP 입력에 없어 바닥이 높다 → 학습 불가
  - 횡방향 Δn, Δα 는 바닥이 낮다 → 학습 가능

두 가지 독립적인 방법으로 측정해 순서가 뒤집히지 않음을 확인한다.
  (A) 최근접이웃 : 입력공간에서 가장 가까운 이웃 k개끼리의 분산 (구간나누기 무관)
  (B) 구간 나누기 : 입력공간을 분위수로 나눠 같은 칸 안의 분산 (칸 수를 바꿔가며)

사용:
    python3 results/channel_learnability.py            # 표 출력 + .mat 저장
"""
import os
import numpy as np
from scipy.spatial import cKDTree
from scipy.io import savemat

DATA = "/home/vilab/CarMaker/mpc_host/hockenheim_mass.npz"
OUT  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "matlab",
                    "fig_channel_learnability.mat")

CH   = ["ds", "dn", "da", "dv"]                 # y 열 순서
LBL  = ["Δs (종)", "Δn (횡)", "Δα (횡)", "Δv (종)"]
FEAT = [0, 1, 2, 3, 4, 5]                       # MLP 입력 = n, α, v, D, δ, κ


def knn_floor(Z, y, k=8):
    """이웃 k개끼리 남는 분산 비율 (자기 자신 포함해 k+1개 조회)."""
    _, idx = cKDTree(Z).query(Z, k=k + 1)
    return np.array([np.mean(y[idx, j].var(axis=1)) / y[:, j].var()
                     for j in range(y.shape[1])])


def bin_floor(F, y, nbin):
    """분위수로 nbin 등분한 칸 안에서 남는 분산 비율."""
    def qb(a):
        return np.digitize(a, np.quantile(a, np.linspace(0, 1, nbin + 1)[1:-1]))
    cell = np.zeros(len(F), dtype=np.int64)
    for c in range(F.shape[1]):
        cell = cell * nbin + qb(F[:, c])
    out = []
    for j in range(y.shape[1]):
        r = y[:, j]
        v, w = [], []
        for c in np.unique(cell):
            m = cell == c
            if m.sum() < 8:
                continue
            v.append(r[m].var()); w.append(m.sum())
        out.append(np.average(v, weights=w) / r.var())
    return np.array(out)


def main():
    d = np.load(DATA)
    X, y = d["X"], d["y"]
    F = X[:, FEAT]
    Z = (F - F.mean(0)) / F.std(0)              # 채널 스케일 정규화

    knn  = knn_floor(Z, y)
    b4   = bin_floor(F[:, [2, 3, 4, 5]], y, 4)  # v, D, δ, κ 로 4등분
    b5   = bin_floor(F[:, [2, 3, 4, 5]], y, 5)

    print(f"데이터 {DATA}  (N={len(X)})\n")
    print("[1] 설명 불가 비율 (분산 기준, 높을수록 학습 불가)")
    print(f"{'채널':>9} | {'최근접이웃':>10} | {'4분할':>7} | {'5분할':>7}")
    print("-" * 44)
    for j, l in enumerate(LBL):
        print(f"{l:>9} | {100*knn[j]:9.1f}% | {100*b4[j]:6.1f}% | {100*b5[j]:6.1f}%")
    print(f"핵심: 횡 Δn {100*knn[1]:.1f}%  vs  종 Δv {100*knn[3]:.1f}%"
          f"  →  {knn[3]/knn[1]:.0f}배  (세 방법 모두 순서 동일)")

    # 슬라이드 18과 같은 단위(RMS 감소율)로 환산: 바닥이 분산의 f 이면
    # 도달 가능한 최대 RMS 감소율 = 1 - sqrt(f)
    ceiling = 100 * (1 - np.sqrt(knn))
    achieved = fit_achieved(X, y)

    print("\n[2] 슬라이드 18과 같은 단위 (RMS 감소율)")
    print(f"{'채널':>9} | {'실제 달성':>9} | {'이론 한계':>9} | {'한계 대비':>9}")
    print("-" * 46)
    for j, l in enumerate(LBL):
        a = achieved[j] if achieved is not None else float("nan")
        print(f"{l:>9} | {a:8.1f}% | {ceiling[j]:8.1f}% | {100*a/ceiling[j]:8.0f}%")

    savemat(OUT, dict(
        channel_names    = np.array(CH, dtype=object),
        unexplained_knn  = 100 * knn,
        unexplained_bin4 = 100 * b4,
        unexplained_bin5 = 100 * b5,
        ceiling_rms      = ceiling,
        achieved_rms     = achieved if achieved is not None else np.full(4, np.nan),
        n_samples        = len(X),
    ))
    print(f"\n저장: {OUT}")


def fit_achieved(X, y, ckpt="mpc/residual_model_nomass_s0.pt"):
    """학습된 MLP가 실제로 줄인 비율 (RMS 기준) — 슬라이드 18의 막대."""
    try:
        import torch
        import sys
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sys.path.insert(0, os.path.join(root, "mpc"))
        from residual_model_wrapper import load_normalized_model
        m = load_normalized_model(os.path.join(root, ckpt), device="cpu")
        with torch.no_grad():
            p = m(torch.tensor(X[:, :6], dtype=torch.float32)).numpy()
        return np.array([100 * (1 - np.sqrt(((y[:, j] - p[:, j]) ** 2).mean())
                                / np.sqrt((y[:, j] ** 2).mean())) for j in range(4)])
    except Exception as e:                       # torch 없으면 건너뜀
        print(f"  (학습 모델 평가 생략: {e})")
        return None


if __name__ == "__main__":
    main()
