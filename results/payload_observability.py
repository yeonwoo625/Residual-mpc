#!/usr/bin/env python3
"""
적재 질량이 왜 MLP 입력으로 불필요한가 — 관측 가능성 + 조건부 잉여성.

슬라이드 25~28은 "무게를 넣어도 성능이 같다"는 *관찰*만 보여준다. 이 스크립트는
그 이유를 정량화한다.

  ① 관측 가능성 : 입력 [n, α, v, D, δ, κ] 만으로 질량을 회귀할 수 있는가?
       → 가능하면 "질량은 상태에 이미 드러나 있다"
  ② 조건부 잉여성 : 입력을 고정했을 때 질량이 잔차를 추가로 설명하는가?
       → 설명력이 0에 가까우면 "질량은 새로운 정보가 아니다"

①만으로는 부족하다(R²<1 이므로 "못 맞히는 부분이 도움될 수도" 라는 반박이 남는다).
②가 그 반박을 닫는다. 둘을 합쳐야 "조건화는 정보이론적으로 잉여" 가 성립한다.

데이터: hockenheim_mass.npz (8,088 샘플, 32/40/48/56 t)
사용:   python3 results/payload_observability.py
"""
import os
import numpy as np
from scipy.spatial import cKDTree
from scipy.io import savemat

HERE  = os.path.dirname(os.path.abspath(__file__))
DATA  = "/home/vilab/CarMaker/mpc_host/hockenheim_mass.npz"
OUT   = os.path.join(HERE, "matlab", "fig_payload_observability.mat")
FEAT  = ["n", "alpha", "v", "D", "delta", "kappa"]
K     = 20


def r2_linear(F, t):
    A = np.c_[F, np.ones(len(F))]
    w, *_ = np.linalg.lstsq(A, t, rcond=None)
    return 1 - ((t - A @ w) ** 2).sum() / ((t - t.mean()) ** 2).sum()


def main():
    d = np.load(DATA)
    X, y, m = d["X"][:, :6], d["y"], d["X"][:, 6]
    Z = (X - X.mean(0)) / X.std(0)

    # ---------- ① 관측 가능성 ----------
    r2_lin = r2_linear(Z, m)
    tree = cKDTree(Z)
    _, idx = tree.query(Z, k=K + 1)
    m_hat = m[idx[:, 1:]].mean(1)                    # 자기 자신 제외
    r2_knn = 1 - ((m - m_hat) ** 2).sum() / ((m - m.mean()) ** 2).sum()

    drop = np.array([r2_lin - r2_linear(Z[:, [j for j in range(6) if j != i]], m)
                     for i in range(6)])

    classes = np.unique(m)
    pred_mean = np.array([m_hat[m == c].mean() for c in classes])
    pred_lo   = np.array([np.percentile(m_hat[m == c], 5) for c in classes])
    pred_hi   = np.array([np.percentile(m_hat[m == c], 95) for c in classes])

    print(f"① 관측 가능성 — 입력 6차원에서 질량 복원   (N={len(X)})")
    print(f"   선형        R² = {r2_lin:.3f}")
    print(f"   최근접이웃  R² = {r2_knn:.3f}   평균오차 {np.abs(m-m_hat).mean()/1000:.1f} t"
          f"   ±4t 적중 {100*np.mean(np.abs(m-m_hat)<4000):.0f}%")
    print("   특징별 기여 (빼면 R² 하락):")
    for f, dv in sorted(zip(FEAT, drop), key=lambda t: -t[1]):
        print(f"     {f:6s} {dv:+.3f}")

    # ---------- ② 조건부 잉여성 ----------
    # 입력이 거의 같은 이웃 묶음 안에서 그룹평균을 뺀 뒤 질량과 잔차의 상관.
    # 그룹평균 제거 = 입력으로 설명되는 성분 제거.
    g_m = m[idx] - m[idx].mean(1, keepdims=True)
    cond = []
    for j in (1, 2):
        g_r = y[idx, j] - y[idx, j].mean(1, keepdims=True)
        c = (g_r * g_m).sum() / np.sqrt((g_r ** 2).sum() * (g_m ** 2).sum())
        cond.append(c ** 2)
    print(f"\n② 조건부 잉여성 — 입력 고정 시 질량의 추가 설명력 (이웃 k={K})")
    print(f"   Δn : {100*cond[0]:.2f}%")
    print(f"   Δα : {100*cond[1]:.2f}%")
    print("   → 입력을 알면 질량은 사실상 새로운 정보가 아니다.")

    savemat(OUT, dict(
        feat_names   = np.array(FEAT, dtype=object),
        r2_linear    = r2_lin,
        r2_knn       = r2_knn,
        mass_err_t   = np.abs(m - m_hat).mean() / 1000,
        hit_4t       = 100 * np.mean(np.abs(m - m_hat) < 4000),
        drop_r2      = drop,                 # (6,) 특징 제외 시 R² 하락
        mass_classes = classes / 1000.0,     # [32 40 48 56] t
        pred_mean    = pred_mean / 1000.0,
        pred_lo      = pred_lo / 1000.0,
        pred_hi      = pred_hi / 1000.0,
        cond_r2_pct  = 100 * np.array(cond), # [Δn, Δα] 추가 설명력 [%]
        k_neighbors  = K,
        n_samples    = len(X),
    ))
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
