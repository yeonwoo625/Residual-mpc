#!/usr/bin/env python3
"""
B1 실험 종합 — nominal 가중치 튜닝 vs residual MPC.

배경: 기존 비교(슬라이드 17/22/23)는 nominal의 비용 가중치를 기본값
(Q_n=1e-4)으로 두고 수행되었다. "튜닝된 nominal과 비교하면?"에 답하기 위해
2026-08-31 에 수행한 실험 전체를 정리한다.

세 갈래:
  A. 속도 스윕   — Q_n=1e-4 고정, 10 / 11.5 / 12 m/s 에서 nominal vs residual
  B. 가중치 스윕 — v=10 고정, Q_n 을 1e-4 ~ 1e-2 (100배) 로 올리며 nominal 단독
  C. 적재 검증   — B 의 최적 gain(1e-2) 하나로 32 / 48 / 56 t
  D. 상호작용    — 높은 gain 에서 잔차 세기(RESIDUAL_SCALE)를 바꿔가며

모든 주행: Hockenheim 1랩, 48t(별도 표기 없으면), 정지 출발, 첫 랩만 평가.
지표는 results/eval_b1.py 와 동일 (급코너 = R<=50m).

사용:  python3 results/b1_summary.py
출력:  표 4개 + results/matlab/fig_b1_summary.mat
"""
import os
import sys
import numpy as np
from scipy.io import savemat

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from eval_b1 import _kappa_fn, evaluate                      # noqa: E402

B1  = os.path.join(HERE, "b1")
OUT = os.path.join(HERE, "matlab", "fig_b1_summary.mat")
kf  = _kappa_fn()


def ev(name):
    return evaluate(os.path.join(B1, name), kf)


def main():
    M = {}

    # ---------- A. 속도 스윕 (Q_n = 1e-4) ----------
    speeds = [(10.0, "v10"), (11.5, "v115"), (12.0, "v12")]
    print("A. 속도 스윕  (Q_n=1e-4 고정, 48t) — nominal vs residual")
    print(f"{'목표v':>6} {'실제v':>7} {'nom':>7} {'res':>7} {'개선':>7} "
          f"{'채터nom':>8} {'채터res':>8}")
    A = {k: [] for k in "v_target v_actual nom res improve chat_nom chat_res".split()}
    for vt, tag in speeds:
        n, r = ev(f"{tag}_nom.npy"), ev(f"{tag}_res.npy")
        imp = 100 * (n["corner_rms"] - r["corner_rms"]) / n["corner_rms"]
        print(f"{vt:6.1f} {r['v_mean']:7.2f} {n['corner_rms']:7.3f} "
              f"{r['corner_rms']:7.3f} {imp:6.1f}% {n['chatter']:8.4f} {r['chatter']:8.4f}")
        for k, v in zip(A, [vt, r["v_mean"], n["corner_rms"], r["corner_rms"],
                            imp, n["chatter"], r["chatter"]]):
            A[k].append(v)
    M["speed"] = {k: np.array(v) for k, v in A.items()}

    # ---------- B. 가중치 스윕 (v = 10, nominal) ----------
    qns = ["1e-4", "3e-4", "1e-3", "3e-3", "1e-2"]
    print("\nB. 가중치 스윕  (v=10, nominal 단독, 48t)")
    print(f"{'Q_n':>7} {'급코너RMS':>10} {'평균|n|':>8} {'채터':>8} {'속도':>7} {'랩타임':>8}")
    B = {k: [] for k in "qn corner mean chatter v laptime".split()}
    for q in qns:
        r = ev(f"qn_{q}.npy")
        print(f"{q:>7} {r['corner_rms']:10.3f} {r['n_mean']:8.3f} {r['chatter']:8.4f} "
              f"{r['v_mean']:7.2f} {r['steps']*0.1:7.1f}s")
        for k, v in zip(B, [float(q), r["corner_rms"], r["n_mean"],
                            r["chatter"], r["v_mean"], r["steps"] * 0.1]):
            B[k].append(v)
    M["qn_sweep"] = {k: np.array(v) for k, v in B.items()}

    # ---------- C. 적재 검증 (Q_n = 1e-2, v = 10) ----------
    loads = [(32, "qn_1e-2_32t.npy"), (48, "qn_1e-2.npy"), (56, "qn_1e-2_56t.npy")]
    print("\nC. 적재 검증  (48t 에서 찾은 Q_n=1e-2 하나로, v=10, nominal)")
    print(f"{'적재':>5} {'급코너RMS':>10} {'평균|n|':>8} {'채터':>8}")
    C = {k: [] for k in "mass corner mean chatter".split()}
    for m_, f in loads:
        r = ev(f)
        print(f"{m_:4d}t {r['corner_rms']:10.3f} {r['n_mean']:8.3f} {r['chatter']:8.4f}")
        for k, v in zip(C, [m_, r["corner_rms"], r["n_mean"], r["chatter"]]):
            C[k].append(v)
    M["payload"] = {k: np.array(v) for k, v in C.items()}

    # ---------- D. 높은 gain 에서의 잔차 세기 ----------
    inter = [("v=10  nominal",      "qn_1e-2.npy",              0.0),
             ("v=10  res s=0.03",   "qn1e-2_v10_res_s003.npy",  0.03),
             ("v=12  nominal",      "qn1e-2_v12_nom.npy",       0.0),
             ("v=12  res s=0.03",   "qn1e-2_v12_res_s003.npy",  0.03),
             ("v=12  res s=0.1",    "qn1e-2_v12_res_s01.npy",   0.1),
             ("v=12  res s=0.3",    "qn1e-2_v12_res_s03.npy",   0.3),
             ("v=12  DAgger s=0.1", "qn1e-2_v12_dagger_s01.npy", 0.1)]
    print("\nD. 높은 gain(Q_n=1e-2)에서 잔차 세기 — 이득이 사라지고 채터만 증가")
    print(f"{'설정':20s} {'급코너RMS':>10} {'평균|n|':>8} {'채터':>8}")
    D = {k: [] for k in "scale corner mean chatter".split()}
    labels = []
    for lbl, f, sc in inter:
        r = ev(f)
        print(f"{lbl:20s} {r['corner_rms']:10.3f} {r['n_mean']:8.3f} {r['chatter']:8.4f}")
        labels.append(lbl)
        for k, v in zip(D, [sc, r["corner_rms"], r["n_mean"], r["chatter"]]):
            D[k].append(v)
    M["interaction"] = {k: np.array(v) for k, v in D.items()}
    M["interaction"]["labels"] = np.array(labels, dtype=object)

    savemat(OUT, M)
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
