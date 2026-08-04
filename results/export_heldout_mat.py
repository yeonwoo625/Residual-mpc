#!/usr/bin/env python3
"""
held-out mass 실험 결과(JSON) -> MATLAB .mat

출력: results/matlab/heldout/heldout_mass.mat
  mass        (4x1)  테스트(제외)한 적재 [kg]
  is_extrap   (4x1)  1 = 외삽(학습범위 밖), 0 = 내삽
  r2_dn_cond    / r2_dn_state    (4x3)  held-out 적재에서의 R^2(dn), 열=시드
  r2_da_cond    / r2_da_state    (4x3)  R^2(dalpha)
  rmse_dn_cond  / rmse_dn_state  (4x3)  RMSE(dn) [m/step]
  r2val_dn_cond / r2val_dn_state (4x3)  in-distribution val R^2(dn)
"""
import os
import json
import numpy as np
from scipy.io import savemat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "results", "heldout_mass_results.json")
OUT = os.path.join(ROOT, "results", "matlab", "heldout")

R = json.load(open(SRC))
masses = sorted({r['held'] for r in R})
seeds = sorted({r['seed'] for r in R})


def grid(model, key, ch=None):
    A = np.full((len(masses), len(seeds)), np.nan)
    for i, m in enumerate(masses):
        for j, s in enumerate(seeds):
            for r in R:
                if r['held'] == m and r['model'] == model and r['seed'] == s:
                    A[i, j] = r[key][ch] if ch else r[key]
    return A


d = {
    'mass': np.array(masses, dtype=float).reshape(-1, 1),
    'is_extrap': np.array([[float(next(r['kind'] for r in R if r['held'] == m) == '외삽')]
                           for m in masses]),
    'seeds': np.array(seeds, dtype=float).reshape(1, -1),
    'r2_dn_cond':     grid('cond',  'r2_test', 'dn'),
    'r2_dn_state':    grid('state', 'r2_test', 'dn'),
    'r2_da_cond':     grid('cond',  'r2_test', 'dalpha'),
    'r2_da_state':    grid('state', 'r2_test', 'dalpha'),
    'rmse_dn_cond':   grid('cond',  'rmse_dn_test'),
    'rmse_dn_state':  grid('state', 'rmse_dn_test'),
    'r2val_dn_cond':  grid('cond',  'r2_val', 'dn'),
    'r2val_dn_state': grid('state', 'r2_val', 'dn'),
    'src': 'results/heldout_mass_results.json',
    'note': ('rows = held-out mass (excluded from training AND from the '
             'normalization stats), cols = init seed'),
}

os.makedirs(OUT, exist_ok=True)
savemat(os.path.join(OUT, "heldout_mass.mat"), d, do_compression=True)
print(f"저장: {os.path.join(OUT, 'heldout_mass.mat')}")
for i, m in enumerate(masses):
    print(f"  {m/1000:.0f}t {'외삽' if d['is_extrap'][i,0] else '내삽'}: "
          f"cond R2(dn) {d['r2_dn_cond'][i].mean():.3f}  "
          f"state {d['r2_dn_state'][i].mean():.3f}")
