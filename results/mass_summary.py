#!/usr/bin/env python3
"""
적재별 종합 — 예측 오차와 주행 오차를 한 표에.

무게를 MLP 입력에 넣을지에 대한 근거를 적재(32/40/48/56 t)별로 모은다. 두 축이 있다.

  가로축 = 무엇을 재나
    예측 오차   1스텝 잔차 예측 오차. 모델 자체의 정확도
    주행 오차   급코너 횡오차 RMS / 헤딩오차 RMS. 폐루프 추종 성능

  세로축 = 그 적재를 학습에서 봤나
    본 적재     32/40/48/56 을 다 섞어 학습. 평가 적재도 학습에 들어있다
    안 본 적재  그 적재를 통째로 빼고 학습(leave-one-mass-out). 처음 보는 적재다

'본 적재' 는 무게에 가장 유리한 조건이다. 학습에서 봤고, 입력에 무게 칸이 있고,
주행 때 정답 무게까지 받는다. 실차에는 적재 중량계가 없으므로 이건 상한선이다.
'안 본 적재' 가 실제 배치 상황에 가깝다.

세 팔을 비교한다.
  nomass    6차원. 무게를 안 준다
  cond      8차원. 무게+CoG 를 준다
  shufmass  8차원 플라시보. 무게 열을 섞어 학습해 정보량이 0 이다.
            cond 가 shufmass 를 못 이기면 이득은 무게가 아니라 입력 차원 탓이다

예측 오차는 results/{mass_conditioning,lomo_mass}.py 가 만든 .mat 에서 읽는다.
주행 오차는 results/ablation/*.npy 를 직접 평가한다.

  본 적재 주행    abl_{nomass,cond,shufmass}_{mass}[_sN].npy      (있음)
  안 본 적재 주행 heldout56/heldout56_{state,cond}_s{N}.npy       (56 t 는 있음)
                  abl_lomo_{nomass,cond}_{mass}_s{N}.npy          (32/40/48 t 는 주행 필요)

56 t 는 2026-08-04 에 이미 주행돼 있다. results/train_heldout56.py 가 32/40/48 t
로만 학습한 모델이고, 가중치·정규화가 lomo_mass.py 의 56 t 모델과 **완전히 동일**함을
확인했다(같은 하이퍼파라미터·시드·분할). 그래서 그대로 '안 본 적재 주행' 으로 쓴다.
이름만 다르다 - heldout56 의 state = nomass, cond = cond.

나머지 적재는 아직 없다. 없으면 NaN 으로 두고 표에 '-' 로 찍는다.
채우려면:  ./scripts/run_ablation.sh lomo_nomass 32 0   (TruckMaker Loads=0)

사용:  python3 results/mass_summary.py
출력:  표 + results/matlab/fig_mass_summary.mat
"""
import os
import sys
import numpy as np
from scipy.io import loadmat, savemat

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "mpc"))
from eval_b1 import _kappa_fn, evaluate                            # noqa: E402

MAT    = os.path.join(HERE, "matlab")
ABL    = os.path.join(HERE, "ablation")
OUT    = os.path.join(MAT, "fig_mass_summary.mat")
MASSES = [32, 40, 48, 56]
ARMS   = ["nomass", "cond", "shufmass"]
SEEDS  = [0, 1, 2]
DN, DA = 1, 2                      # 채널: 0=ds 1=dn 2=dalpha 3=dv
RAD    = 180 / np.pi
# 본 적재 주행의 조건. 40 t 만 다르다 - 절대값 비교 불가, 쌍비교만 유효
NOTE   = {32: "v=10, 6.5Hz, 3 seeds", 40: "v=12, 10Hz, 1 seed",
          48: "v=10, 6.5Hz, 3 seeds", 56: "v=10, 6.5Hz, 3 seeds"}


def drive(path, kf):
    if not os.path.exists(path):
        return None
    r = evaluate(path, kf)
    return [r["corner_rms"], r["a_corner"]]


def main():
    kf = _kappa_fn()

    # ---------- 예측 오차: 기존 .mat 에서 읽는다 ----------
    mc = loadmat(os.path.join(MAT, "fig_mass_conditioning.mat"), squeeze_me=True)
    lo = loadmat(os.path.join(MAT, "fig_lomo_mass.mat"), squeeze_me=True)
    # mass_conditioning 의 variant 순서: 0=보정없음 1=nomass 2=cond 3=shufmass
    P_SEEN   = mc["pred_err"][:, 1:, :, :]        # (적재, 팔3, 시드, 채널)
    P_RAW    = mc["pred_err"][:, 0, 0, :]         # (적재, 채널) 보정 없음
    P_UNSEEN = lo["err"]                          # (적재, 팔3, 시드, 채널)
    P_URAW   = lo["err_raw"]

    # ---------- 주행 오차: .npy 를 직접 평가 ----------
    D_SEEN   = np.full((len(MASSES), 3, len(SEEDS), 2), np.nan)
    D_UNSEEN = np.full((len(MASSES), 3, len(SEEDS), 2), np.nan)
    for mi, m in enumerate(MASSES):
        for ai, arm in enumerate(ARMS):
            for si, s in enumerate(SEEDS):
                suf = "" if (s == 0 and arm != "shufmass") else f"_s{s}"
                v = drive(os.path.join(ABL, f"abl_{arm}_{m}{suf}.npy"), kf)
                if v:
                    D_SEEN[mi, ai, si] = v
                if arm == "shufmass":             # 플라시보는 LOMO 로 안 만든다
                    continue
                # 56 t 는 heldout56 이름으로 이미 주행돼 있다 (모델은 동일)
                old = {"nomass": "state", "cond": "cond"}[arm]
                cands = [os.path.join(ABL, f"abl_lomo_{arm}_{m}_s{s}.npy"),
                         os.path.join(ABL, f"abl_lomo_{arm}_{m}{suf}.npy")]
                if m == 56:
                    cands.append(os.path.join(HERE, "heldout56",
                                              f"heldout56_{old}_s{s}.npy"))
                for p in cands:
                    v = drive(p, kf)
                    if v:
                        D_UNSEEN[mi, ai, si] = v
                        break

    # ---------- 표 ----------
    def cell(a):
        a = a[~np.isnan(a)]
        return f"{a.mean():8.4f}±{a.std():.4f}" if len(a) else f"{'-':>15}"

    def gain(a, b):
        a, b = a[~np.isnan(a)], b[~np.isnan(b)]
        if not len(a) or not len(b):
            return f"{'-':>9}"
        return f"{100*(a.mean()-b.mean())/a.mean():8.1f}%"

    for lbl, ch, sc, uP, uD in (("횡  (Δn / |n| RMS)", DN, 1.0, "m", "m"),
                                ("헤딩 (Δα / α RMS)", DA, RAD, "deg", "deg")):
        print("\n" + "=" * 92)
        print(f"{lbl}   — 3시드 평균±표준편차")
        print("=" * 92)
        rows = [("예측 오차", "본 적재",    P_RAW,  P_SEEN,   ch,          uP),
                ("예측 오차", "안 본 적재", P_URAW, P_UNSEEN, ch,          uP),
                ("주행 오차", "본 적재",    None,   D_SEEN,   0 if ch == DN else 1, uD),
                ("주행 오차", "안 본 적재", None,   D_UNSEEN, 0 if ch == DN else 1, uD)]
        for what, seen, RAW, A, ci, unit in rows:
            k = sc if what == "예측 오차" else 1.0
            print(f"\n  [{what} / {seen}]  단위 {unit}")
            print(f"{'적재':>6}{'보정없음':>11}{'무게 X':>17}{'무게 O':>17}"
                  f"{'섞은무게':>17}{'이득':>10}")
            for mi, m in enumerate(MASSES):
                r0 = f"{RAW[mi, ci]*k:11.4f}" if RAW is not None else f"{'-':>11}"
                a = [A[mi, ai, :, ci] * k for ai in range(3)]
                note = ""
                if what == "주행 오차" and seen == "본 적재":
                    note = "  " + NOTE[m]
                print(f"{m:5d}t{r0}{cell(a[0]):>17}{cell(a[1]):>17}"
                      f"{cell(a[2]):>17}{gain(a[0], a[1])}{note}")

    savemat(OUT, dict(
        mass       = np.array(MASSES),
        arm        = np.array(["without mass", "with mass", "shuffled mass"], dtype=object),
        seen       = np.array(["seen payload", "unseen payload"], dtype=object),
        pred_chan  = np.array(["ds", "dn", "dalpha", "dv"], dtype=object),
        drive_met  = np.array(["corner n RMS [m]", "corner alpha RMS [deg]"], dtype=object),
        pred_seen  = P_SEEN,       # (적재, 팔, 시드, 채널)
        pred_unseen= P_UNSEEN,
        pred_raw_seen   = P_RAW,   # (적재, 채널) 보정 없음
        pred_raw_unseen = P_URAW,
        drive_seen  = D_SEEN,      # (적재, 팔, 시드, 지표)
        drive_unseen= D_UNSEEN,
        drive_note  = np.array([NOTE[m] for m in MASSES], dtype=object),
        drive_same  = np.array([m != 40 for m in MASSES]),   # 절대값 비교 가능 여부
    ))
    n_u = int(np.sum(~np.isnan(D_UNSEEN[:, :, :, 0])))
    print(f"\n안 본 적재 주행: {n_u}회 확보"
          + ("" if n_u else "  (아직 없음 - run_ablation.sh lomo_nomass|lomo_cond)"))
    print(f"저장: {OUT}")


if __name__ == "__main__":
    main()
