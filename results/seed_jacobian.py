#!/usr/bin/env python3
"""
시드 간 재현성: 잔차의 '값'은 식별되지만 '미분'은 식별되지 않는다.

같은 데이터·같은 구조·같은 하이퍼파라미터로, 오직 난수 초기값(seed)만 바꿔
5개 모델을 학습한 뒤 서로 비교한다.

  - 값 Δ 가 시드마다 같다        → 데이터가 값을 결정한다 (식별됨)
  - 미분 ∂Δ/∂x 가 시드마다 다르다 → 데이터가 미분을 결정하지 못한다 (식별 안 됨)

미분이 초기 난수에 좌우된다는 것은 그것이 트럭의 물리가 아니라 학습의 우연이라는 뜻이며,
이것이 first-order 주입(λ≠0)이 발산하는 근본 원인이다.

불일치 지표 = (시드 간 표준편차의 평균) / (샘플 간 표준편차)
             = 재현되지 않는 비율. 0%면 완전 재현, 100%면 신호만큼 흔들림.

사용:
    python3 results/seed_jacobian.py
"""
import os
import sys
import numpy as np
import torch
from scipy.io import savemat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "mpc"))
from residual_model_wrapper import load_normalized_model   # noqa: E402

DATA   = "/home/vilab/CarMaker/mpc_host/hockenheim_nomass.npz"
MODELS = os.path.join(ROOT, "results", "seed_models")
OUT    = os.path.join(ROOT, "results", "matlab", "fig_seed_jacobian.mat")

STATES   = ["n", "alpha", "v", "D", "delta", "kappa"]
CHANNELS = ["ds", "dn", "da", "dv"]
NSAMPLE  = 2000


def disagreement(A):
    """A: (S, N, ...) -> 시드 간 불일치 [%]"""
    return 100 * A.std(0).mean(0) / A.mean(0).std(0)


def main():
    paths = sorted(p for p in os.listdir(MODELS) if p.endswith(".pt"))
    if len(paths) < 2:
        sys.exit(f"모델이 부족합니다 ({MODELS}). 먼저 시드별 학습을 돌리세요.")
    models = [load_normalized_model(os.path.join(MODELS, p), device="cpu") for p in paths]

    X = np.load(DATA)["X"][:, :6]
    idx = np.random.default_rng(0).choice(len(X), min(NSAMPLE, len(X)), replace=False)
    Xs = X[idx]

    VAL, JAC = [], []
    for m in models:
        xt = torch.tensor(Xs, dtype=torch.float32, requires_grad=True)
        out = m(xt)
        VAL.append(out.detach().numpy())
        J = np.zeros((len(Xs), 4, 6))
        for k in range(4):
            g, = torch.autograd.grad(out[:, k].sum(), xt, retain_graph=(k < 3))
            J[:, k, :] = g.detach().numpy()
        JAC.append(J)
    VAL, JAC = np.array(VAL), np.array(JAC)      # (S,N,4) / (S,N,4,6)

    d_val = disagreement(VAL)                    # (4,)
    d_jac = disagreement(JAC)                    # (4,6)

    S = len(models)
    print(f"시드 {S}개, 샘플 {len(Xs)}개  —  같은 데이터/구조, 초기값만 다름\n")
    print("[값]  Δ")
    for k, c in enumerate(CHANNELS):
        print(f"   {c:3s} : {d_val[k]:6.1f}%")
    print("\n[미분]  ∂Δn/∂x   (MPC에 주입되는 바로 그 값)")
    for j, s in enumerate(STATES):
        print(f"   d(dn)/d({s:5s}) : {d_jac[1, j]:6.1f}%")
    print(f"\n요약: 값 Δn {d_val[1]:.1f}%  vs  미분 평균 {d_jac[1].mean():.1f}%"
          f"   →  {d_jac[1].mean()/d_val[1]:.0f}배")

    savemat(OUT, dict(
        n_seeds       = S,
        n_samples     = len(Xs),
        state_names   = np.array(STATES, dtype=object),
        channel_names = np.array(CHANNELS, dtype=object),
        value_disagree = d_val,          # (4,)   채널별 값 불일치 [%]
        jac_disagree   = d_jac,          # (4,6)  채널×상태 미분 불일치 [%]
        dn_value       = d_val[1],       # 그림에 쓰는 두 숫자
        dn_jac         = d_jac[1],       # (6,)
    ))
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
