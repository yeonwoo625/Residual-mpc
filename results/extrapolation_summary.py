#!/usr/bin/env python3
"""
외삽 일반화 — 학습 분포 밖에서도 잔차 보정이 작동하는가.

배경: 일반화 축이 둘 있었으나 둘 다 **내삽**이었다.
  · 적재(32/48/56 t)  — 전부 학습 데이터에 포함
  · 미학습 트랙(USA Highway No.1) — 트랙은 새롭지만 상태는 학습 범위 안쪽이다.
    핵심 4차원(n, α, δ, κ)이 전부 학습 범위 내부이고 곡률은 오히려 훨씬 완만하다
    (최소반경 Hockenheim 40 m vs Highway 107 m). 6차원 최근접거리로 재면
    Highway(중앙값 0.693)가 **같은 트랙에서 속도만 바꾼 주행(1.043)보다 가깝다.**
    즉 "새로운 도로"가 아니라 "더 완만한 도로"였다.

따라서 학습 범위를 실제로 벗어나는 축이 필요했다. 속도를 골랐다 — 트랙 제작이
필요 없고, 학습 데이터의 속도 상한(p99 = 12.03 m/s)이 명확하기 때문이다.

설정: Hockenheim 건조, 48 t, Q_n=1e-4, 잔차 scale 0.3, 재학습 없음
      (`mpc/residual_model_nomass_s0.pt`, 학습 데이터 `ablation/hockenheim_nomass.npz`).
      목표 14 m/s. alat_max=5.0 이 최급코너 속도를 13.7 m/s 로 묶으므로
      목표를 16/18 로 올려도 코너 속도는 같다 — 직선만 빨라진다.

결과: 주행 상태의 80~92% 가 학습 분포 밖인 조건에서 급코너 오차 43.4% 감소,
      조향 채터 38% 감소, 양쪽 완주.

외삽 지표는 **방향이 있는 것**을 쓴다. 학습 데이터의 속도 최대값(12.234 m/s)을
넘는 샘플 비율과, 도달 최고속도의 초과율이다.

거리 기반 지표(정규화 6차원 최근접거리)는 쓰지 않는다. 방향을 구분하지 못하기
때문이다 — v=10 주행은 학습 데이터(대부분 11.9 m/s 부근)에서 멀지만 **더 느려서**
먼 것이므로 외삽이 아니다. 실제로 같은 거리 지표로 재면 v=10 이 96%,  v=14 가
80~92% 로 순서가 뒤집힌다. 참고용으로 거리 자체는 표에 남기되 외삽 비율로
해석하지 않는다.

사용:  python3 results/extrapolation_summary.py
출력:  표 2개 + results/matlab/fig_extrapolation.mat
"""
import os
import sys
import numpy as np
from scipy.io import savemat
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from eval_b1 import _kappa_fn, evaluate                      # noqa: E402

B1    = os.path.join(HERE, "b1")
TRAIN = os.path.join(HERE, "ablation", "hockenheim_nomass.npz")
OUT   = os.path.join(HERE, "matlab", "fig_extrapolation.mat")
SPEEDS = [(10.0, "v10"), (11.5, "v115"), (12.0, "v12"), (14.0, "v14")]
LBL = ["n", "alpha", "v", "D", "delta", "kappa"]


def feats(path, kf):
    """궤적 -> 잔차 모델 입력 6차원. 출발 가속 구간(v<2)은 제외."""
    a = np.load(path)
    w = np.where(np.diff(a[:, 0]) < -100)[0]
    if len(w):
        a = a[:w[0] + 1]
    s, n, al, v, D, de = a.T
    X = np.stack([n, al, v, D, de, kf(s)], 1)
    return X[v > 2.0]


def main():
    kf = _kappa_fn()
    tr = np.load(TRAIN)["X"]
    mu, sd = tr.mean(0), tr.std(0)
    tree = cKDTree((tr - mu) / sd)
    # 학습 데이터 내부 최근접거리의 99% 분위 = "이보다 멀면 외삽"
    thr = np.percentile(tree.query((tr - mu) / sd, k=2)[0][:, 1], 99)
    v_hi = tr[:, 2].max()            # 학습에서 실제로 본 최고 속도

    print(f"학습 데이터: {len(tr)} 샘플, 속도 최대 = {v_hi:.3f} m/s "
          f"(p99 {np.percentile(tr[:, 2], 99):.2f})\n")

    print("A. 속도 스윕 — 성능")
    print(f"{'목표v':>6}{'실제v':>7}{'완주':>7}{'nom':>8}{'res':>8}{'개선':>8}"
          f"{'채터nom':>9}{'채터res':>9}")
    A = {k: [] for k in
         "v_target v_actual nom res improve chat_nom chat_res".split()}
    for vt, tag in SPEEDS:
        n = evaluate(os.path.join(B1, f"{tag}_nom.npy"), kf)
        r = evaluate(os.path.join(B1, f"{tag}_res.npy"), kf)
        imp = 100 * (n["corner_rms"] - r["corner_rms"]) / n["corner_rms"]
        done = ("O" if n["done"] else "X") + "/" + ("O" if r["done"] else "X")
        print(f"{vt:6.1f}{r['v_mean']:7.2f}{done:>7}{n['corner_rms']:8.3f}"
              f"{r['corner_rms']:8.3f}{imp:7.1f}%{n['chatter']:9.4f}{r['chatter']:9.4f}")
        for k, val in zip(A, [vt, r["v_mean"], n["corner_rms"], r["corner_rms"],
                              imp, n["chatter"], r["chatter"]]):
            A[k].append(val)

    print("\nB. 학습 속도 상한(최대 %.2f m/s)을 얼마나 넘었나" % v_hi)
    print(f"{'주행':>16}{'평균v':>7}{'최대v':>7}{'상한대비':>9}{'상한초과':>9}"
          f"{'(참고)NN':>10}")
    B = {k: [] for k in "label v_mean v_max frac_v nn_med frac_ood".split()}
    for vt, tag in SPEEDS:
        for kind in ("nom", "res"):
            X = feats(os.path.join(B1, f"{tag}_{kind}.npy"), kf)
            d = tree.query((X - mu) / sd, k=1)[0]
            lbl = f"v{vt:g} {kind}"
            row = [X[:, 2].mean(), X[:, 2].max(),
                   100 * np.mean(X[:, 2] > v_hi), np.median(d),
                   100 * np.mean(d > thr)]
            print(f"{lbl:>16}{row[0]:7.2f}{row[1]:7.2f}"
                  f"{100 * (row[1] / v_hi - 1):+8.1f}%{row[2]:8.1f}%{row[3]:10.3f}")
            B["label"].append(lbl)
            for k, val in zip("v_mean v_max frac_v nn_med frac_ood".split(), row):
                B[k].append(val)

    print("\nC. 차원별 범위 — v=14 residual 이 학습 범위를 어디서 벗어나나")
    X = feats(os.path.join(B1, "v14_res.npy"), kf)
    C = {"label": LBL, "train_lo": [], "train_hi": [],
         "run_lo": [], "run_hi": [], "frac_out": []}
    print(f"{'':>7}{'학습 1~99%':>24}{'v14 res 범위':>24}{'밖':>8}")
    for j, l in enumerate(LBL):
        lo, hi = np.percentile(tr[:, j], [1, 99])
        o = 100 * np.mean((X[:, j] < lo) | (X[:, j] > hi))
        print(f"{l:>7}  [{lo:+8.3f},{hi:+8.3f}]  [{X[:, j].min():+8.3f},"
              f"{X[:, j].max():+8.3f}]{o:7.1f}%")
        for k, val in zip("train_lo train_hi run_lo run_hi frac_out".split(),
                          [lo, hi, X[:, j].min(), X[:, j].max(), o]):
            C[k].append(val)

    i = A["v_target"].index(14.0)
    print(f"\n핵심: 최고속도가 학습 상한을 {100 * (B['v_max'][-1] / v_hi - 1):.0f}% 초과하고 "
          f"주행의 {B['frac_v'][-1]:.0f}%(nominal {B['frac_v'][-2]:.0f}%)가 상한 밖인 "
          f"조건에서\n      급코너 오차 {A['improve'][i]:.1f}% 감소, "
          f"채터 {100 * (1 - A['chat_res'][i] / A['chat_nom'][i]):.0f}% 감소, 양쪽 완주.")
    print("      거리 지표는 방향을 구분하지 못하므로(v=10 도 96%) 외삽 근거로 쓰지 않는다.")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    # 속도 분포 (그림 (c) 용) — 공통 격자에서의 히스토그램
    edges = np.linspace(2.0, 15.0, 66)
    ctr = 0.5 * (edges[:-1] + edges[1:])
    # 변수명 hist 는 MATLAB 내장 함수와 겹치므로 vdist 로 저장한다
    hists = {"centers": ctr,
             "train": np.histogram(tr[:, 2], edges, density=True)[0]}
    for tag in ("v12_res", "v14_nom", "v14_res"):
        Xh = feats(os.path.join(B1, f"{tag}.npy"), kf)
        hists[tag] = np.histogram(Xh[:, 2], edges, density=True)[0]

    savemat(OUT, {
        "vdist": hists,
        "speed":  {k: np.array(v) for k, v in A.items()},
        "ood":    {k: (np.array(v, dtype=object) if k == "label" else np.array(v))
                   for k, v in B.items()},
        "dims":   {k: (np.array(v, dtype=object) if k == "label" else np.array(v))
                   for k, v in C.items()},
        "train_v_max": v_hi,
        "nn_threshold": thr,
    })
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
