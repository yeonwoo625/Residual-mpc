#!/usr/bin/env python3
"""
Held-out mass 실험: 무게 조건화가 '학습에 없던 적재'로 일반화되는가?

각 적재를 하나씩 완전히 빼고 학습한 뒤, 그 적재에서 잔차 예측 성능을 비교한다.
  cond       : 8차원 입력 [n,a,v,D,d,kappa, mass, cog]  (ConditionedResidualModel, concat)
  state-only : 6차원 입력 [n,a,v,D,d,kappa]             (ResidualModel)
두 모델은 입력 차원 말고 모든 것이 같다 (구조/하이퍼파라미터/시드/조기종료).

핵심 규칙: 조기종료용 val 은 '학습에 쓰는 적재'에서만 뗀다.
           테스트 적재는 학습에도 val 에도 절대 등장하지 않는다.

사용:
    mpc_host/venv/bin/python results/heldout_mass.py
"""
import os
import sys
import json
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # mpc_docker/
sys.path.insert(0, os.path.join(ROOT, "mpc"))
from residual_model import ResidualModel, ConditionedResidualModel

DATA = os.path.join(os.path.dirname(ROOT), "mpc_host", "hockenheim_mass.npz")
OUT = os.path.join(ROOT, "results", "heldout_mass_results.json")

EPOCHS, BATCH, LR, HIDDEN = 200, 64, 1e-3, 64
SEEDS = [0, 1, 2]
CH = ["ds", "dn", "dalpha", "dv"]


def r2(y_true, y_pred):
    """채널별 R^2 (물리 단위, 테스트셋 평균 기준)."""
    ss_res = ((y_true - y_pred) ** 2).sum(axis=0)
    ss_tot = ((y_true - y_true.mean(axis=0)) ** 2).sum(axis=0)
    return 1.0 - ss_res / ss_tot


def train_one(Xtr, ytr, Xva, yva, cond, seed):
    """한 모델 학습 -> (best_state 로 복원된 model, norm)."""
    torch.manual_seed(seed)
    np.random.seed(seed)

    norm = dict(X_mean=Xtr.mean(0), X_std=Xtr.std(0) + 1e-6,
                y_mean=ytr.mean(0), y_std=ytr.std(0) + 1e-6)
    nz = lambda a, m, s: (a - m) / s
    Xtr_n = torch.tensor(nz(Xtr, norm['X_mean'], norm['X_std']), dtype=torch.float32)
    ytr_n = torch.tensor(nz(ytr, norm['y_mean'], norm['y_std']), dtype=torch.float32)
    Xva_n = torch.tensor(nz(Xva, norm['X_mean'], norm['X_std']), dtype=torch.float32)
    yva_n = torch.tensor(nz(yva, norm['y_mean'], norm['y_std']), dtype=torch.float32)

    if cond:
        model = ConditionedResidualModel(state_dim=6, context_dim=Xtr.shape[1] - 6,
                                         output_dim=4, hidden_dim=HIDDEN, mode='concat')
    else:
        model = ResidualModel(input_dim=6, output_dim=4, hidden_dim=HIDDEN, n_layers=3)

    opt = torch.optim.Adam(model.parameters(), lr=LR)      # weight_decay=0 (원 학습과 동일)
    crit = nn.MSELoss()
    loader = DataLoader(TensorDataset(Xtr_n, ytr_n), batch_size=BATCH, shuffle=True)

    best, best_state = float('inf'), None
    for _ in range(EPOCHS):
        model.train()
        for xb, yb in loader:
            opt.zero_grad()
            crit(model(xb), yb).backward()
            opt.step()
        model.eval()
        with torch.no_grad():
            vl = crit(model(Xva_n), yva_n).item()
        if vl < best:
            best = vl
            best_state = {k: v.clone() for k, v in model.state_dict().items()}
    model.load_state_dict(best_state)
    return model, norm


def predict(model, norm, X):
    with torch.no_grad():
        Xn = torch.tensor((X - norm['X_mean']) / norm['X_std'], dtype=torch.float32)
        yn = model(Xn).numpy()
    return yn * norm['y_std'] + norm['y_mean']


def main():
    d = np.load(DATA)
    X, y = d['X'].astype(np.float64), d['y'].astype(np.float64)
    mass = X[:, 6]
    masses = np.unique(mass)
    print(f"데이터 {len(X)} 샘플, 적재 {[f'{m/1000:.0f}t' for m in masses]}\n")

    results = []
    for held in masses:
        te = mass == held
        tr_all = ~te
        # 조기종료용 val: 학습 적재에서만 20% (테스트 적재는 절대 안 씀)
        rng = np.random.default_rng(42)
        idx = np.where(tr_all)[0]
        rng.shuffle(idx)
        nval = int(0.2 * len(idx))
        va_idx, tr_idx = idx[:nval], idx[nval:]

        inside = masses.min() < held < masses.max()
        kind = "내삽" if inside else "외삽"
        print(f"=== held-out {held/1000:.0f}t ({kind}) | train {len(tr_idx)} "
              f"val {len(va_idx)} test {te.sum()} ===", flush=True)

        for cond in (True, False):
            cols = slice(None) if cond else slice(0, 6)
            for seed in SEEDS:
                model, norm = train_one(X[tr_idx][:, cols], y[tr_idx],
                                        X[va_idx][:, cols], y[va_idx], cond, seed)
                p_te = predict(model, norm, X[te][:, cols])
                p_va = predict(model, norm, X[va_idx][:, cols])
                rt, rv = r2(y[te], p_te), r2(y[va_idx], p_va)
                rmse_dn = float(np.sqrt(((y[te][:, 1] - p_te[:, 1]) ** 2).mean()))
                results.append(dict(held=float(held), kind=kind,
                                    model='cond' if cond else 'state', seed=seed,
                                    r2_test={c: float(v) for c, v in zip(CH, rt)},
                                    r2_val={c: float(v) for c, v in zip(CH, rv)},
                                    rmse_dn_test=rmse_dn))
                print(f"  {'cond ' if cond else 'state'} s{seed}: "
                      f"test R2(dn)={rt[1]:+.3f} R2(da)={rt[2]:+.3f} | "
                      f"val R2(dn)={rv[1]:+.3f} | RMSE(dn)={rmse_dn:.4f}", flush=True)

    with open(OUT, 'w') as f:
        json.dump(results, f, indent=1)
    print(f"\n저장: {OUT}")

    # ---- 요약 ----
    print("\n=== 요약: held-out 적재에서의 R^2(dn), 3시드 평균 ===")
    print(f"{'held':>6} {'종류':>5} {'cond':>16} {'state-only':>16} {'차이(cond-state)':>18}")
    for held in masses:
        g = lambda mdl: np.array([r['r2_test']['dn'] for r in results
                                  if r['held'] == held and r['model'] == mdl])
        c, s = g('cond'), g('state')
        kind = [r['kind'] for r in results if r['held'] == held][0]
        print(f"{held/1000:5.0f}t {kind:>5} {c.mean():8.3f}±{c.std():.3f} "
              f"{s.mean():8.3f}±{s.std():.3f} {c.mean()-s.mean():+18.3f}")


if __name__ == "__main__":
    main()
