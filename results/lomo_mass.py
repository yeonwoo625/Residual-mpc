#!/usr/bin/env python3
"""
안 본 적재로 일반화되는가 — leave-one-mass-out.

기존 조건화 ablation(results/mass_conditioning.py)은 32/40/48/56 t 를 **전부 섞어**
학습한 뒤 그중 20%로 검증했다. 즉 주행하는 적재를 학습에서 이미 봤다. 그래서
"안 본 적재에서도 되는가" 라는 질문에는 답하지 못한다.

여기서는 적재 하나를 통째로 빼고 학습한 뒤, 그 적재에서만 평가한다.

  학습 풀   나머지 세 적재 (이 안에서 다시 80/20 으로 나눠 val 은 best epoch 선택용)
  시험      빼놓은 적재 전체. 학습에도 val 에도 한 번도 안 들어간다.

세 팔을 같은 시드로 학습한다. 차이는 입력의 무게 열뿐이다.

  nomass    6차원. 무게를 안 준다
  cond      8차원. 무게+CoG 를 준다. 시험 때 **학습에 없던 무게값**을 받는다
  shufmass  8차원. 무게 열을 학습 풀 안에서 섞었다 — 입력 차원은 같고 정보만 없는
            플라시보. cond 와 shufmass 가 같으면 이득은 무게가 아니라 차원 탓이다

지표는 1스텝 예측 오차(잔차 보정 후 남은 오차)의 채널별 RMS 다. 폐루프 주행은
필요 없다 — 트럭메이커를 돌리지 않는다.

사용:  python3 results/lomo_mass.py
출력:  표 + results/matlab/fig_lomo_mass.mat
"""
import os
import sys
import numpy as np
import torch
import torch.nn as nn
from scipy.io import savemat

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "mpc"))
from residual_model import ResidualModel, ConditionedResidualModel   # noqa: E402

DATA   = "/home/vilab/CarMaker/mpc_host/hockenheim_mass.npz"
OUT    = os.path.join(HERE, "matlab", "fig_lomo_mass.mat")
MASSES = [32, 40, 48, 56]
ARMS   = ["nomass", "cond", "shufmass"]
SEEDS  = [0, 1, 2]
# 학습 하이퍼파라미터 — mpc/train_residual.py 기본값과 동일하게 맞춘다
EPOCHS, BATCH, LR, HIDDEN, LAYERS = 200, 64, 1e-3, 64, 3
VAL_RATIO, SPLIT_SEED = 0.2, 42


def build(arm, seed):
    torch.manual_seed(seed)
    if arm == "nomass":
        return ResidualModel(input_dim=6, output_dim=4,
                             hidden_dim=HIDDEN, n_layers=LAYERS)
    return ConditionedResidualModel(state_dim=6, context_dim=2, output_dim=4,
                                    hidden_dim=HIDDEN, mode="concat")


def fit(Xtr, ytr, Xva, yva, arm, seed, dev):
    """학습 풀로 학습, val 로 best epoch 선택. 정규화는 학습 풀로만 맞춘다."""
    xm, xs = Xtr.mean(0), Xtr.std(0) + 1e-6
    ym, ys = ytr.mean(0), ytr.std(0) + 1e-6
    tt = lambda a: torch.tensor(a, dtype=torch.float32, device=dev)
    Xt, yt = tt((Xtr - xm) / xs), tt((ytr - ym) / ys)
    Xv, yv = tt((Xva - xm) / xs), tt((yva - ym) / ys)

    torch.manual_seed(seed)
    np.random.seed(seed)
    model = build(arm, seed).to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=LR)
    crit = nn.MSELoss()
    loader = torch.utils.data.DataLoader(
        torch.utils.data.TensorDataset(Xt, yt), batch_size=BATCH, shuffle=True)

    best, best_state = float("inf"), None
    for _ in range(EPOCHS):
        model.train()
        for xb, yb in loader:
            opt.zero_grad(); crit(model(xb), yb).backward(); opt.step()
        model.eval()
        with torch.no_grad():
            vl = crit(model(Xv), yv).item()
        if vl < best:
            best, best_state = vl, {k: v.clone() for k, v in model.state_dict().items()}
    model.load_state_dict(best_state)
    return model, (xm, xs, ym, ys)


def test_rms(model, norm, Xte, yte, dev):
    """시험 적재에서 보정 후 남은 오차의 채널별 RMS (원단위)."""
    xm, xs, ym, ys = norm
    model.eval()
    with torch.no_grad():
        g = model(torch.tensor((Xte - xm) / xs, dtype=torch.float32, device=dev))
    g = g.cpu().numpy() * ys + ym
    return np.sqrt(((yte - g) ** 2).mean(0))


def main():
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    d = np.load(DATA)
    X, y = d["X"], d["y"]
    mv = np.round(X[:, 6] / 1000).astype(int)
    print(f"데이터 {len(X)} 샘플, 입력 {X.shape[1]}차원, 장치 {dev}")
    print("  적재별: " + ", ".join(f"{m}t:{int((mv == m).sum())}" for m in MASSES))
    print(f"\n적재 하나를 통째로 빼고 학습 -> 그 적재에서만 평가."
          f"  {len(MASSES)}적재 x {len(ARMS)}팔 x {len(SEEDS)}시드 = "
          f"{len(MASSES)*len(ARMS)*len(SEEDS)}회 학습\n")

    # E[mass, arm, seed, channel] / E0[mass, channel] = 보정 없음
    E  = np.full((len(MASSES), len(ARMS), len(SEEDS), 4), np.nan)
    E0 = np.full((len(MASSES), 4), np.nan)

    for mi, H in enumerate(MASSES):
        te = mv == H
        Xte, yte = X[te], y[te]
        E0[mi] = np.sqrt((yte ** 2).mean(0))

        pool = ~te
        Xp, yp = X[pool], y[pool]
        n_val = int(len(Xp) * VAL_RATIO)
        idx = np.random.default_rng(SPLIT_SEED).permutation(len(Xp))
        va, tr = idx[:n_val], idx[n_val:]
        print(f"[{H}t 제외] 학습 {len(tr)} / val {len(va)} / 시험 {int(te.sum())}", flush=True)

        for ai, arm in enumerate(ARMS):
            Xtr, Xva = Xp[tr].copy(), Xp[va].copy()
            if arm == "nomass":
                Xtr, Xva, Xt2 = Xtr[:, :6], Xva[:, :6], Xte[:, :6]
            else:
                Xt2 = Xte
            for si, s in enumerate(SEEDS):
                if arm == "shufmass":       # 무게 열만 학습 풀 안에서 섞는다
                    r = np.random.default_rng(1000 + s)
                    Xtr = Xp[tr].copy(); Xva = Xp[va].copy()
                    Xtr[:, 6:8] = Xtr[r.permutation(len(Xtr)), 6:8]
                    Xva[:, 6:8] = Xva[r.permutation(len(Xva)), 6:8]
                m, nrm = fit(Xtr, yp[tr], Xva, yp[va], arm, s, dev)
                E[mi, ai, si] = test_rms(m, nrm, Xt2, yte, dev)
            print(f"    {arm:9} 완료", flush=True)

    # ---------- 표 ----------
    for ch, name, unit, sc in ((1, "횡 Δn", "m", 1.0), (2, "헤딩 Δα", "deg", 180 / np.pi)):
        print("\n" + "=" * 78)
        print(f"안 본 적재에서의 예측 오차 — [{name}] 단위 {unit}, 3시드 평균±표준편차")
        print("=" * 78)
        print(f"{'뺀적재':>7}{'보정없음':>11}{'무게 X':>18}{'무게 O':>18}"
              f"{'섞은무게':>18}{'조건화이득':>11}")
        for mi, H in enumerate(MASSES):
            c = [E[mi, ai, :, ch] * sc for ai in range(3)]
            g = 100 * (c[0].mean() - c[1].mean()) / c[0].mean()
            print(f"{H:6d}t{E0[mi, ch]*sc:11.4f}" +
                  "".join(f"{a.mean():12.4f}±{a.std():.4f}" for a in c) +
                  f"{g:10.1f}%")
        n, cd = E[:, 0, :, ch].ravel(), E[:, 1, :, ch].ravel()
        dd = n - cd
        print(f"  전체 12쌍: 평균차(무게X-무게O) {dd.mean()*sc:+.5f} {unit}, "
              f"무게O 가 나은 쌍 {int((dd > 0).sum())}/12")

    savemat(OUT, dict(
        mass    = np.array(MASSES),
        arm     = np.array(["without mass", "with mass", "shuffled mass"], dtype=object),
        channel = np.array(["ds", "dn", "dalpha", "dv"], dtype=object),
        err     = E,      # (mass, arm, seed, channel)  안 본 적재 예측 오차
        err_raw = E0,     # (mass, channel)             보정 없음
        seeds   = np.array(SEEDS),
    ))
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
