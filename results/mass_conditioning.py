#!/usr/bin/env python3
"""
적재 조건화 근거 — MLP 입력에 무게를 넣어야 하는가.

이 연구는 잔차 MLP 입력에 무게를 **넣지 않았다**(6차원 상태만). 그 결정의 근거를
적재 32 / 40 / 48 / 56 t 에서 조건화(8차원, 무게+CoG 포함) 모델과 나란히 비교해
정리한다.

두 층위에서 본다.

  ① 예측 오차 (모델 자체)  — 무게를 알려주면 잔차를 더 잘 맞히는가
     학습에 쓰지 않은 검증 분할에서 1스텝 예측 오차를 적재별로 잰다.
       nominal   보정 없음. 오차 = 잔차 y 자체의 RMS
       nomass    6차원 모델로 보정한 뒤 남은 오차
       cond      8차원(무게+CoG) 모델로 보정한 뒤 남은 오차
     채널은 횡방향 Δn 과 헤딩 Δα 를 본다.

  ② 폐루프 추종 오차 (실주행) — 그 차이가 주행 성능으로 이어지는가
     results/ablation/ 의 주행에서 급코너 횡오차 RMS 와 헤딩오차 RMS.

  ③ 플라시보 (48 t) — 폐루프 차이가 정말 '무게 정보' 때문인가
     무게 라벨을 무작위로 섞어 학습한 모델(shufmass). 입력 차원은 8차원으로
     같지만 무게 정보는 없다. 이것이 진짜 무게 모델만큼 잘하면, 폐루프에서
     관측된 차이는 무게 정보가 아니라 입력 차원/초기값/학습 노이즈 탓이다.

**한계를 먼저 밝힌다.** 폐루프에서 **40 t 는 주행 조건이 다르다.**
32 / 48 / 56 t 는 `TARGET_SPEED=10`, 솔버 수정 전(SQP 100회 -> 실제 6.5 Hz)에
시드 3개씩 돌았고, 40 t 는 `TARGET_SPEED=12`, 수정 후(10 Hz)에 **시드 0 하나만**
돌았다(2026-09-03). 따라서 40 t 의 **절대값은 다른 적재와 비교할 수 없다** —
속도가 1.9 m/s 빨라 오차가 약 2배다. 같은 조건에서 돌린 **40 t 내부의
무게 O/X 쌍비교만** 유효하다.

모델: mpc/residual_model_{cond,nomass}_s{0,1,2}.pt  (시드만 다르고 나머지 동일)
데이터: /home/vilab/CarMaker/mpc_host/hockenheim_mass.npz (8,088 샘플, 8차원)
        results/ablation/hockenheim_nomass.npz 는 여기서 mass 열만 뺀 것이다.
분할: train_residual.load_dataset 과 동일 (random, seed=42, val 20%).

사용:  python3 results/mass_conditioning.py
출력:  표 3개 + results/matlab/fig_mass_conditioning.mat
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
from residual_model_wrapper import load_normalized_model          # noqa: E402
from eval_b1 import _kappa_fn, evaluate                           # noqa: E402

DATA   = "/home/vilab/CarMaker/mpc_host/hockenheim_mass.npz"
ABL    = os.path.join(HERE, "ablation")
OUT    = os.path.join(HERE, "matlab", "fig_mass_conditioning.mat")
SEEDS  = [0, 1, 2]
MASSES = [32, 40, 48, 56]
VAL_RATIO, SPLIT_SEED = 0.2, 42


def val_split():
    """train_residual.load_dataset(split_mode='random', seed=42) 와 동일한 검증 분할."""
    d = np.load(DATA)
    X, y = d["X"], d["y"]
    n_val = int(len(X) * VAL_RATIO)
    idx = np.random.default_rng(SPLIT_SEED).permutation(len(X))
    return X[idx][:n_val], y[idx][:n_val]


def pred_err(Xv, yv, model, use_mass):
    """보정 후 남은 1스텝 예측 오차. 반환 (4,) 채널별 RMS."""
    feat = Xv if use_mass else Xv[:, :6]
    with torch.no_grad():
        g = model(torch.from_numpy(feat.astype(np.float32))).numpy()
    return np.sqrt(((yv - g) ** 2).mean(0))


def main():
    Xv, yv = val_split()
    mv = np.round(Xv[:, 6] / 1000).astype(int)
    print(f"검증 분할 {len(Xv)} 샘플 (학습에 쓰지 않음)")
    print("  적재별: " + ", ".join(f"{m}t:{int((mv == m).sum())}" for m in MASSES))

    models = {}
    for kind, use_mass in (("cond", True), ("nomass", False), ("shufmass", True)):
        for s in SEEDS:
            p = os.path.join(ROOT, "mpc", f"residual_model_{kind}_s{s}.pt")
            models[(kind, s)] = (load_normalized_model(p, device="cpu"), use_mass)

    # ---------- ① 예측 오차 ----------
    # E[mass, variant, seed, channel]
    #   variant 0=보정없음 1=nomass 2=cond 3=shufmass(플라시보)
    E = np.full((len(MASSES), 4, len(SEEDS), 4), np.nan)
    for mi, m in enumerate(MASSES):
        sel = mv == m
        E[mi, 0, :] = np.sqrt((yv[sel] ** 2).mean(0))          # 보정 없음
        for si, s in enumerate(SEEDS):
            for vi, kind in ((1, "nomass"), (2, "cond"), (3, "shufmass")):
                mdl, um = models[(kind, s)]
                E[mi, vi, si] = pred_err(Xv[sel], yv[sel], mdl, um)

    print("\n" + "=" * 78)
    print("① 1스텝 예측 오차 (검증 분할) — 보정 후 남은 오차, 3시드 평균±표준편차")
    print("=" * 78)
    for ch, name, unit, sc in ((1, "횡 Δn", "m", 1.0), (2, "헤딩 Δα", "deg", 180 / np.pi)):
        print(f"\n  [{name}]  단위 {unit}")
        print(f"{'적재':>6}{'보정없음':>11}{'무게 X':>18}{'무게 O':>18}{'섞은무게':>18}{'조건화이득':>11}")
        for mi, m in enumerate(MASSES):
            e0 = E[mi, 0, 0, ch] * sc
            n_m, n_s = E[mi, 1, :, ch].mean() * sc, E[mi, 1, :, ch].std() * sc
            c_m, c_s = E[mi, 2, :, ch].mean() * sc, E[mi, 2, :, ch].std() * sc
            p_m, p_s = E[mi, 3, :, ch].mean() * sc, E[mi, 3, :, ch].std() * sc
            gain = 100 * (n_m - c_m) / n_m
            print(f"{m:5d}t{e0:11.4f}{n_m:12.4f}±{n_s:.4f}{c_m:12.4f}±{c_s:.4f}"
                  f"{p_m:12.4f}±{p_s:.4f}{gain:10.1f}%")

    # ---------- ② 폐루프 ----------
    kf = _kappa_fn()
    CL_M = [32, 40, 48, 56]
    # 주행 조건 (40t 만 다르다 - 절대값 비교 불가, 쌍비교만 유효)
    CL_NOTE = {32: "v=10, 6.5Hz, 3 seeds", 40: "v=12, 10Hz, 1 seed",
               48: "v=10, 6.5Hz, 3 seeds", 56: "v=10, 6.5Hz, 3 seeds"}
    # variant 0=nomass 1=cond 2=shufmass(48t 만 존재)
    C = np.full((len(CL_M), 3, len(SEEDS), 2), np.nan)
    for mi, m in enumerate(CL_M):
        for vi, kind in enumerate(("nomass", "cond", "shufmass")):
            for si, s in enumerate(SEEDS):
                suf = "" if (s == 0 and kind != "shufmass") else f"_s{s}"
                p = os.path.join(ABL, f"abl_{kind}_{m}{suf}.npy")
                if not os.path.exists(p):
                    continue
                r = evaluate(p, kf)
                C[mi, vi, si] = [r["corner_rms"], r["a_corner"]]

    print("\n" + "=" * 78)
    print("② 폐루프 추종 오차 (실주행) — 3시드 평균±표준편차")
    print("=" * 78)
    print(f"{'적재':>6}{'지표':>15}{'무게 X':>17}{'무게 O':>17}"
          f"{'섞은무게':>17}{'이득':>8}  주행조건")
    for mi, m in enumerate(CL_M):
        for ci, lbl in enumerate(("급코너 |n| RMS", "급코너 α RMS")):
            cells, means = [], []
            for vi in range(3):
                a = C[mi, vi, :, ci]; a = a[~np.isnan(a)]
                means.append(a.mean() if len(a) else np.nan)
                cells.append(f"{a.mean():8.3f}±{a.std():.3f}" if len(a) else f"{'-':>14}")
            g = 100 * (means[0] - means[1]) / means[0]
            note = CL_NOTE[m] if ci == 0 else ""
            print(f"{(str(m)+'t') if ci == 0 else '':>6}{lbl:>15}"
                  f"{cells[0]:>17}{cells[1]:>17}{cells[2]:>17}{g:7.1f}%  {note}")

    print("\n③ 플라시보 판정 (48 t)")
    a = C[CL_M.index(48)]
    for ci, lbl in enumerate(("급코너 |n| RMS", "급코너 α RMS")):
        nm = np.nanmean(a[0, :, ci]); cd = np.nanmean(a[1, :, ci]); sh = np.nanmean(a[2, :, ci])
        frac = 100 * (nm - sh) / (nm - cd) if nm != cd else np.nan
        print(f"  {lbl}: 무게없음 {nm:.3f} -> 진짜무게 {cd:.3f}, 섞은무게 {sh:.3f}")
        print(f"      섞은 무게가 '조건화 이득'의 {frac:.0f}% 를 재현한다.")

    print("\n한계")
    print("  · 40 t 폐루프는 v=12 / 10 Hz / 시드1개로 조건이 다르다.")
    print("    절대값은 다른 적재와 비교 불가. 40 t 내부 쌍비교만 유효하다.")
    print("  · 플라시보는 48 t 만 학습·주행돼 있다.")

    savemat(OUT, dict(
        mass_pred   = np.array(MASSES),
        variant     = np.array(["no correction", "without mass", "with mass",
                                "shuffled mass"], dtype=object),
        channel     = np.array(["ds", "dn", "dalpha", "dv"], dtype=object),
        pred_err    = E,                    # (mass, variant, seed, channel)
        mass_cloop  = np.array(CL_M),
        cloop_var   = np.array(["without mass", "with mass", "shuffled mass"], dtype=object),
        cloop_metric= np.array(["corner n RMS [m]", "corner alpha RMS [deg]"], dtype=object),
        cloop_note  = np.array([CL_NOTE[m] for m in CL_M], dtype=object),
        cloop_same  = np.array([m != 40 for m in CL_M]),   # 절대값 비교 가능 여부
        cloop_err   = C,                    # (mass, variant, seed, metric)
        n_val       = len(Xv),
    ))
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
