#!/usr/bin/env python3
"""
저마찰 노면 실험 — 잔차 보정의 작동 조건 규명.

배경: 마른 노면(μ=1.0)에서는 nominal 의 비용 가중치 튜닝만으로 잔차의 이득을
대부분 대체할 수 있었다(results/b1/). 잔차가 실제로 필요한 영역이 있는지
확인하기 위해, kinematic 모델의 '슬립 0' 가정이 크게 깨지는 저마찰에서 검증했다.

설정: Hockenheim, Link.0.Friction = 0.3 (도로파일 DEU_Hockenheim_mu05.rd5),
      48 t, Q_n=1e-4, 잔차 모델은 해당 노면에서 재수집·재학습(4,978 샘플).

핵심 발견: 잔차 보정은 **타이어에 여유가 있을 때만** 작동한다.
  마찰 사용률 73% (v=9)  -> 급코너 오차 34.0% 감소, 완주
  마찰 사용률 90% (v=10) -> 조향 ±30° 포화, 4회 시도 전부 이탈

기전: 잔차는 "예측보다 더 밀리므로 더 조향하라"는 보정을 준다. 한계 영역에서는
추가 조향이 횡력으로 변환되지 않으므로(조향 실효이득 0.65 -> 0.37) 보정이
악순환을 만든다. nominal 은 밀리는 것을 모르므로 덜 꺾어 살아남지만 6 m 밀린다.

사용:  python3 results/friction_summary.py
출력:  표 2개 + results/matlab/fig_friction.mat
"""
import os
import sys
import numpy as np
from scipy.io import savemat

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from eval_b1 import _kappa_fn, evaluate                    # noqa: E402

MU03 = os.path.join(HERE, "mu03")
OUT  = os.path.join(HERE, "matlab", "fig_friction.mat")
DT, L, RMIN, MU = 0.1, 4.32, 1 / 0.0265, 0.3


def steer_gain(path, kf):
    """조향 실효이득 = 실제 요레이트 / kinematic 모델 예측, 급코너 구간 중앙값."""
    a = np.load(path)
    w = np.where(np.diff(a[:, 0]) < -100)[0]
    a = a[:w[0] + 1] if len(w) else a
    s, al, v, de = a[:, 0], a[:, 2], a[:, 3], a[:, 5]
    psidot = np.gradient(al, DT) + kf(s) * np.gradient(s, DT)
    c = (np.abs(kf(s)) >= 0.02) & (np.abs(de) > 0.05) & (v > 5)
    vc = v[c].mean()
    return (np.median(psidot[c] / (v[c] * de[c] / L)),
            100 * np.mean(np.abs(np.rad2deg(de)) > 28),
            vc, vc * vc / RMIN)


def main():
    kf = _kappa_fn()
    runs = [  # (파일, 라벨, 목표속도, 잔차여부)
        ("mu03_nom.npy",         "v=10 nominal",            10, 0),
        ("mu03_res.npy",         "v=10 res (마른학습 s0.3)",  10, 1),
        ("mu03_res_trained.npy", "v=10 res (재학습 s0.3)",    10, 1),
        ("mu03_res_s01.npy",     "v=10 res (재학습 s0.1)",    10, 1),
        ("mu03_res_rd.npy",      "v=10 res (R_DELTA×10)",    10, 1),
        ("mu03_v9_nom.npy",      "v=9  nominal",              9, 0),
        ("mu03_v9_res.npy",      "v=9  res (재학습 s0.3)",     9, 1),
    ]
    print("μ = 0.3 저마찰 주행 (48 t, Q_n=1e-4)\n")
    print(f"{'설정':26s} {'완주':>5} {'거리':>7} {'급코너RMS':>9} {'평균|n|':>8} "
          f"{'사용률':>6} {'실효이득':>7} {'포화':>6}")
    print("-" * 88)
    rows = []
    for f, lbl, vt, isres in runs:
        p = os.path.join(MU03, f)
        if not os.path.exists(p):
            continue
        r = evaluate(p, kf)
        g, sat, vc, ay = steer_gain(p, kf)
        util = 100 * ay / (MU * 9.81)
        print(f"{lbl:26s} {'O' if r['done'] else 'X':>5} {r['dist']:6.0f}m "
              f"{r['corner_rms']:9.3f} {r['n_mean']:8.3f} {util:5.0f}% {g:7.2f} {sat:5.1f}%")
        rows.append(dict(label=lbl, v_target=vt, is_res=isres, done=int(r["done"]),
                         dist=r["dist"], corner=r["corner_rms"], mean=r["n_mean"],
                         chatter=r["chatter"], util=util, gain=g, sat=sat))

    ok = {r["v_target"]: {} for r in rows}
    for r in rows:
        if r["done"]:
            ok[r["v_target"]]["res" if r["is_res"] else "nom"] = r["corner"]
    print("\n완주한 조건에서의 nominal vs residual")
    for vt in sorted(ok):
        d = ok[vt]
        if "nom" in d and "res" in d:
            print(f"  v={vt}: {d['nom']:.3f} -> {d['res']:.3f}  "
                  f"({100*(d['nom']-d['res'])/d['nom']:+.1f}%)")
        elif "nom" in d:
            print(f"  v={vt}: nominal {d['nom']:.3f} 완주, residual 전부 이탈")

    savemat(OUT, dict(
        mu           = MU,
        labels       = np.array([r["label"] for r in rows], dtype=object),
        v_target     = np.array([r["v_target"] for r in rows]),
        is_residual  = np.array([r["is_res"] for r in rows]),
        completed    = np.array([r["done"] for r in rows]),
        distance_m   = np.array([r["dist"] for r in rows]),
        corner_rms   = np.array([r["corner"] for r in rows]),
        mean_n       = np.array([r["mean"] for r in rows]),
        chatter      = np.array([r["chatter"] for r in rows]),
        friction_util= np.array([r["util"] for r in rows]),
        steer_gain   = np.array([r["gain"] for r in rows]),
        steer_sat_pct= np.array([r["sat"] for r in rows]),
    ))
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
