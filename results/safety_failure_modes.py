#!/usr/bin/env python3
"""
안전성 / 실패 모드 정리 — 학습 기반 제어기에 반드시 필요한 절의 근거.

학습 잔차는 학습 분포 안에서만 신뢰할 수 있다. 이 스크립트는
(A) 이미 구현되어 있는 포화 한계(clamp)가 실제로 얼마나 작동하는지 측정하고,
(B) 지금까지 관측된 실패 모드를 한 표로 모은다.

--- (A) 포화 한계 ---
mpc/residual_model_wrapper.py 는 잔차 출력에 물리적 상한을 건다:
    clamp = [Δs 0.2, Δn 0.1, Δα 0.05, Δv 0.4]   (0.1 s 스텝당, m·rad)
분포 밖 입력에서 작은 MLP 가 발산적으로 외삽하는 것을 막는 안전망이다.
실제 제어에 주입되는 것은 B 행렬에 의해 Δn, Δα 뿐이므로 (Δs, Δv 행은 0),
Δn 의 포화 빈도가 실질적인 지표다.

주목할 점: 포화 빈도가 정상 주행에서는 5~7% 인데 발산한 주행에서는 3배로
올라간다. 즉 **런타임 이상 징후 지표**로 쓸 수 있다.

--- (B) 실패 모드 ---
네 가지가 관측되었으며 모두 정량 근거가 있다. 상세는 표 참조.

사용:  python3 results/safety_failure_modes.py
출력:  표 2개 + results/matlab/fig_safety.mat
"""
import os
import sys
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "mpc"))
sys.path.insert(0, HERE)
from residual_model_wrapper import load_normalized_model      # noqa: E402
from eval_b1 import _kappa_fn, evaluate                       # noqa: E402
from scipy.io import savemat                                  # noqa: E402

CLAMP = np.array([0.2, 0.1, 0.05, 0.4])
CH    = ["ds", "dn", "da", "dv"]
OUT   = os.path.join(HERE, "matlab", "fig_safety.mat")

RUNS = [  # (파일, 라벨, 완주여부 설명)
    ("b1/v10_res.npy",              "v=10  (논문 주 결과)",   "완주"),
    ("b1/v115_res.npy",             "v=11.5",                "완주"),
    ("b1/v12_res.npy",              "v=12  (82% 외삽)",      "완주"),
    ("b1/qn1e-2_v12_res_s01.npy",   "v=12  고게인 scale 0.1", "완주"),
    ("b1/qn1e-2_v12_res_s03.npy",   "v=12  고게인 scale 0.3", "발산 이력"),
]

FAILURE_MODES = [
    ("분포 밖 외삽",   "주행의 82%가 학습 속도 상한(12.03 m/s) 초과",
     "급코너 개선 57.9% -> 17.4%",            "results/b1/, b1_summary.py"),
    ("게인 상호작용",  "Q_n 100배 + 잔차 scale 0.3",
     "조향 채터 0.095 -> 0.495, 도로 이탈",   "results/b1/qn1e-2_v12_res_s03"),
    ("미분 주입",      "선형화에 잔차 Jacobian 포함 (λ != 0)",
     "크기·부호 무관 전부 발산 (~16 s)",       "results/logs/, figM1"),
    ("미분 비식별성",  "같은 데이터·구조, 초기값만 다른 5개 모델",
     "값 3.5% vs 미분 78% 불일치",            "results/seed_jacobian.py"),
]


def main():
    kf = _kappa_fn()
    m  = load_normalized_model(os.path.join(ROOT, "mpc", "residual_model_nomass_s0.pt"),
                               device="cpu")

    print("(A) 포화 한계(clamp) 작동 빈도")
    print(f"    한계 = [Δs {CLAMP[0]}, Δn {CLAMP[1]}, Δα {CLAMP[2]}, Δv {CLAMP[3]}]"
          "  (0.1 s 스텝당)")
    print(f"    ※ 제어에 실제 주입되는 것은 Δn, Δα 뿐 (B 행렬이 Δs·Δv 행을 0 으로)\n")
    print(f"{'주행':26s} {'상태':>9} {'Δn 포화':>9} {'Δα 포화':>9} {'채터':>9}")
    print("-" * 68)
    labels, hits_dn, hits_da, chats = [], [], [], []
    for f, lbl, note in RUNS:
        p = os.path.join(HERE, f)
        a = np.load(p)
        w = np.where(np.diff(a[:, 0]) < -100)[0]
        a = a[:w[0] + 1] if len(w) else a
        X = np.column_stack([a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5], kf(a[:, 0])])
        with torch.no_grad():
            Y = m(torch.tensor(X, dtype=torch.float32)).numpy()
        h = [100 * np.mean(np.abs(Y[:, j]) >= CLAMP[j] * 0.999) for j in range(4)]
        ch = evaluate(p, kf)["chatter"]
        print(f"{lbl:26s} {note:>9} {h[1]:8.1f}% {h[2]:8.1f}% {ch:9.4f}")
        labels.append(lbl); hits_dn.append(h[1]); hits_da.append(h[2]); chats.append(ch)

    print("\n    → 정상 주행 5~7%, 발산 이력 주행 18.6% (약 3배).")
    print("      포화 빈도는 런타임 이상 징후 지표로 사용할 수 있다.")

    print("\n(B) 관측된 실패 모드")
    print(f"{'모드':16s} {'조건':38s} {'증상':32s} 근거")
    print("-" * 118)
    for a_, b_, c_, d_ in FAILURE_MODES:
        print(f"{a_:16s} {b_:38s} {c_:32s} {d_}")

    savemat(OUT, dict(
        clamp        = CLAMP,
        clamp_names  = np.array(CH, dtype=object),
        run_labels   = np.array(labels, dtype=object),
        sat_dn_pct   = np.array(hits_dn),
        sat_da_pct   = np.array(hits_da),
        chatter      = np.array(chats),
        fail_mode    = np.array([r[0] for r in FAILURE_MODES], dtype=object),
        fail_cond    = np.array([r[1] for r in FAILURE_MODES], dtype=object),
        fail_symptom = np.array([r[2] for r in FAILURE_MODES], dtype=object),
    ))
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
