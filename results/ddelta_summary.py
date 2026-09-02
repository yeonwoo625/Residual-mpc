#!/usr/bin/env python3
"""
조향 각속도 제약 민감도 — 성능 이득이 완화된 제약에 기인하는가?

배경: MPC 의 조향 각속도 제약이 model.ddelta_max = 1.0 rad/s = 57.3 deg/s
(앞바퀴)로 설정되어 있었다. scripts/record_steer.py 로 측정한 결과 동일
시뮬레이터의 기준 운전자(IPGDriver)가 같은 코스에서 사용한 최대값은 2.47 deg/s
로, 기본 제약은 그 23배다. 차량 모델(Steering.Kind = GenAngle)에는 조향 속도
한계가 정의되어 있지 않아 시뮬레이터에서 스펙을 얻을 수 없다.

따라서 특정 값을 주장하는 대신 **제약을 3.8배 강화(57.3 -> 15 deg/s)했을 때
결론이 유지되는지**를 확인했다. 이는 "비현실적으로 완화된 제약 덕분에 나온
결과 아니냐"는 반박에 대한 직접적인 답이다.

결과: 개선율 유지(58->54%, 17->19%), 조향 채터는 39~47% 감소.
      즉 성능 이득은 제약 완화에 기인하지 않으며, 현실적 제약에서 오히려
      조향 평활도가 개선된다.

사용:  python3 results/ddelta_summary.py
출력:  표 + results/matlab/fig_ddelta.mat
"""
import os
import sys
import numpy as np
from scipy.io import savemat

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from eval_b1 import _kappa_fn, evaluate                     # noqa: E402

LOOSE, TIGHT = 57.3, 15.0          # deg/s (앞바퀴)
IPG_MAX = 2.47                     # IPGDriver 최대, results/ipgdriver_steer.npy


def main():
    kf = _kappa_fn()
    # (속도, 제약, 파일)  — loose 는 기존 results/b1/, tight 는 results/ddelta/
    runs = [(10, LOOSE, "b1/v10_nom.npy",     "b1/v10_res.npy"),
            (10, TIGHT, "ddelta/v10_nom.npy", "ddelta/v10_res.npy"),
            (12, LOOSE, "b1/v12_nom.npy",     "b1/v12_res.npy"),
            (12, TIGHT, "ddelta/v12_nom.npy", "ddelta/v12_res.npy")]

    print(f"조향 각속도 제약 민감도  (앞바퀴 기준, IPGDriver 최대 {IPG_MAX} deg/s)\n")
    print(f"{'속도':>5} {'제약':>8} {'nom':>7} {'res':>7} {'개선':>7} "
          f"{'채터nom':>9} {'채터res':>9}")
    print("-" * 60)
    V, L, NOM, RES, IMP, CN, CR = [], [], [], [], [], [], []
    for v, lim, fn, fr in runs:
        n = evaluate(os.path.join(HERE, fn), kf)
        r = evaluate(os.path.join(HERE, fr), kf)
        imp = 100 * (n["corner_rms"] - r["corner_rms"]) / n["corner_rms"]
        print(f"{v:4d} {lim:7.1f}° {n['corner_rms']:7.3f} {r['corner_rms']:7.3f} "
              f"{imp:6.1f}% {n['chatter']:9.4f} {r['chatter']:9.4f}")
        V.append(v); L.append(lim); NOM.append(n["corner_rms"]); RES.append(r["corner_rms"])
        IMP.append(imp); CN.append(n["chatter"]); CR.append(r["chatter"])

    V, L = np.array(V), np.array(L)
    IMP, CN, CR = np.array(IMP), np.array(CN), np.array(CR)
    print("\n제약 강화(57.3 -> 15 deg/s)의 효과")
    for v in (10, 12):
        a = (V == v) & (L == LOOSE); b = (V == v) & (L == TIGHT)
        print(f"  v={v}: 개선율 {IMP[a][0]:.1f}% -> {IMP[b][0]:.1f}%   "
              f"채터(res) {CR[a][0]:.4f} -> {CR[b][0]:.4f} ({100*(CR[b][0]/CR[a][0]-1):+.0f}%)")

    savemat(os.path.join(HERE, "matlab", "fig_ddelta.mat"), dict(
        limit_loose=LOOSE, limit_tight=TIGHT, ipg_driver_max=IPG_MAX,
        v_target=V, limit=L, nom=np.array(NOM), res=np.array(RES),
        improve=IMP, chat_nom=CN, chat_res=CR,
    ))
    print(f"\n저장: {os.path.join(HERE,'matlab','fig_ddelta.mat')}")


if __name__ == "__main__":
    main()
