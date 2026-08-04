#!/usr/bin/env python3
"""
56 t 를 완전히 제외하고 두 모델을 학습 -> 폐루프 주행용 .pt 저장.

  models/residual_heldout56_cond.pt    8차원 (mass, cog 입력 O)
  models/residual_heldout56_state.pt   6차원 (mass 입력 X)

학습 = 32/40/48 t 만. 56 t 는 학습·val·정규화 통계 어디에도 안 들어간다.
시드 3개 학습 후 val 손실이 가장 낮은 것을 저장 (val 도 학습 적재에서만 뗀 것이라
테스트 적재 정보는 선택 과정에도 개입하지 않음).

사용: mpc_host/venv/bin/python results/train_heldout56.py
"""
import os
import sys
import numpy as np
import torch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "mpc"))
sys.path.insert(0, os.path.join(ROOT, "results"))
from heldout_mass import train_one, predict, r2, SEEDS, HIDDEN     # 같은 학습 루틴 재사용

DATA = os.path.join(os.path.dirname(ROOT), "mpc_host", "hockenheim_mass.npz")
OUTDIR = os.path.join(ROOT, "models")
HELD = 56000.0


def main():
    d = np.load(DATA)
    X, y = d['X'].astype(np.float64), d['y'].astype(np.float64)
    mass = X[:, 6]

    te = mass == HELD
    rng = np.random.default_rng(42)
    idx = np.where(~te)[0]
    rng.shuffle(idx)
    nval = int(0.2 * len(idx))
    va_idx, tr_idx = idx[:nval], idx[nval:]
    print(f"학습 적재 {[f'{m/1000:.0f}t' for m in np.unique(mass[tr_idx])]}  "
          f"train {len(tr_idx)}  val {len(va_idx)}  (56t {te.sum()}개는 완전 제외)\n")

    os.makedirs(OUTDIR, exist_ok=True)
    # 시드마다 따로 저장한다: 폐루프 주행을 시드 3개씩 돌려 run-to-run 산포를
    # 정량화해야 "조건화 차이 없음"을 통계로 말할 수 있다.
    for cond in (True, False):
        cols = slice(None) if cond else slice(0, 6)
        name = 'cond' if cond else 'state'
        in_dim = 8 if cond else 6
        for seed in SEEDS:
            model, norm = train_one(X[tr_idx][:, cols], y[tr_idx],
                                    X[va_idx][:, cols], y[va_idx], cond, seed)
            pv = predict(model, norm, X[va_idx][:, cols])
            rv = r2(y[va_idx], pv)
            rt = r2(y[te], predict(model, norm, X[te][:, cols]))
            path = os.path.join(OUTDIR, f"residual_heldout56_{name}_s{seed}.pt")
            torch.save({
                'state_dict': model.state_dict(),
                'norm': {k: np.asarray(v) for k, v in norm.items()},
                'config': {
                    'input_dim': in_dim, 'output_dim': 4,
                    'hidden_dim': HIDDEN, 'n_layers': 3,
                    'state_dim': 6, 'context_dim': in_dim - 6,
                    'mode': 'concat' if cond else None,
                },
            }, path)
            print(f"  {name:5s} s{seed}: R2val(dn) {rv[1]:+.3f} | 미학습 56t R2(dn) "
                  f"{rt[1]:+.3f}  -> {os.path.basename(path)}", flush=True)


if __name__ == "__main__":
    main()
