#!/usr/bin/env python3
"""
적재 조건화 실험 전체(A~D)를 MATLAB .mat 으로 내보낸다.

출력: results/matlab/runs/<name>.mat   (각 주행 = s,n,alpha,v,D,delta + raw/cols/src)
  nom_{32,48,56}                 nominal MPC (잔차 없음)
  blind_{32,48,56}_s{0,1,2}      무게-blind 잔차 (6dim)
  cond_{32,48,56}_s{0,1,2}       무게 조건화 잔차 (8dim)
  shufmass_48_s{0,1,2}           가짜 무게 플라시보 (8dim, 정보량 0)
  heldout56_{blind,cond}_s{0,1,2}  56t 를 학습에서 뺀 모델로 56t 주행

통계는 MATLAB 쪽 lap_stats.m 이 raw 에서 다시 계산한다 (정의 단일화).
"""
import os
import numpy as np
from scipy.io import savemat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # mpc_docker/
CM = os.path.dirname(ROOT)
HOST = os.path.join(CM, "mpc_host")
ABL = os.path.join(ROOT, "results", "ablation")
RES = os.path.join(ROOT, "results")
OUT = os.path.join(ROOT, "results", "matlab", "runs")
COLS = ["s", "n", "alpha", "v", "D", "delta"]


def first_existing(*paths):
    for p in paths:
        if os.path.exists(p):
            return p
    return None


def jobs():
    J = {}
    for m in (32, 48, 56):
        J[f"nom_{m}"] = os.path.join(RES, f"traj_nom_{m}.npy")
        for arm, tag in (("nomass", "blind"), ("cond", "cond")):
            for s in (0, 1, 2):
                sfx = "" if s == 0 else f"_s{s}"
                J[f"{tag}_{m}_s{s}"] = first_existing(
                    os.path.join(ABL, f"abl_{arm}_{m}{sfx}.npy"),      # 기존(32/56 + 48 s0)
                    os.path.join(ABL, f"abl_{arm}_{m}_s{s}.npy"),      # 저장소 안 사본
                    os.path.join(HOST, f"abl_{arm}_{m}_s{s}.npy"),     # 수집 직후 위치
                )
    HELD = os.path.join(RES, "heldout56")
    for s in (0, 1, 2):
        J[f"shufmass_48_s{s}"] = first_existing(
            os.path.join(ABL, f"abl_shufmass_48_s{s}.npy"),
            os.path.join(HOST, f"abl_shufmass_48_s{s}.npy"))
        J[f"heldout56_blind_s{s}"] = first_existing(
            os.path.join(HELD, f"heldout56_state_s{s}.npy"),
            os.path.join(HOST, f"heldout56_state_s{s}.npy"))
        J[f"heldout56_cond_s{s}"] = first_existing(
            os.path.join(HELD, f"heldout56_cond_s{s}.npy"),
            os.path.join(HOST, f"heldout56_cond_s{s}.npy"))
    return J


def main():
    os.makedirs(OUT, exist_ok=True)
    n_ok, missing = 0, []
    for name, path in sorted(jobs().items()):
        if path is None or not os.path.exists(path):
            missing.append(name)
            continue
        a = np.load(path).astype(np.float64)
        d = {c: a[:, i].reshape(-1, 1) for i, c in enumerate(COLS)}
        d["raw"] = a
        d["cols"] = np.array(COLS, dtype=object).reshape(1, -1)
        d["src"] = os.path.relpath(path, CM)
        savemat(os.path.join(OUT, f"{name}.mat"), d, do_compression=True)
        n_ok += 1
        print(f"  runs/{name}.mat  ({a.shape[0]}x{a.shape[1]})  <- {os.path.relpath(path, CM)}")
    print(f"\n{n_ok}개 저장 -> {OUT}")
    if missing:
        print(f"누락 {len(missing)}개: {', '.join(missing)}")


if __name__ == "__main__":
    main()
