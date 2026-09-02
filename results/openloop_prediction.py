#!/usr/bin/env python3
"""
개루프 예측 오차 — 잔차가 '모델'을 실제로 개선하는가.

배경: 지금까지의 증거는 전부 **폐루프 추종 오차**(주행했더니 덜 벗어났다)다.
그런데 이 연구의 주장은 "학습 잔차가 kinematic 모델의 오차를 보정한다"이므로,
제어기를 거치지 않은 **모델 정확도 자체**를 보여야 주장과 증거가 맞물린다.
폐루프 결과만으로는 "MPC 튜닝 효과 아니냐"는 반박을 막을 수 없다.

방법: 기록된 주행에서 상태 x_i 와 이후 입력열 u_i..u_{i+N-1} 을 그대로 주고
N 스텝을 **개루프로 적분**해 실제 x_{i+N} 과 비교한다. 제어기는 개입하지 않는다.

  nominal          x_{k+1} = x_k + dt * f_nominal(x_k, u_k)
  +residual        x_{k+1} = 위 + B @ g(x_k)        g = 학습된 잔차 MLP

B 는 두 가지를 본다.
  deployed  Δn, Δα 만 (mpc_solver_residual.B_MATRIX. 실제 주행에 쓰는 설정)
  all       Δs, Δn, Δα, Δv 전부 (모델이 낼 수 있는 최대 개선)

N = 20 (2.0 s) 은 MPC 예측구간과 같다. 즉 "MPC 가 매 스텝 내다보는 그 구간에서
모델이 얼마나 틀리는가"를 직접 재는 값이다.

주의: 잔차 모델(residual_model_nomass_s0.pt)의 학습 데이터는 Hockenheim
v=12 부근(hockenheim_nomass.npz, 속도 최대 12.23 m/s)이다. v=14 / Highway /
저마찰은 학습에 없던 조건이므로 개선폭이 줄거나 뒤집힐 수 있다 — 그것을 보는
것이 이 표의 목적이다.

사용:  python3 results/openloop_prediction.py
출력:  표 + results/matlab/fig_openloop.mat
"""
import os
import sys
import numpy as np
import torch
from scipy.io import savemat
from scipy.interpolate import make_interp_spline

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "mpc"))
from dynamics_predict import predict_next_state                # noqa: E402
from residual_model_wrapper import load_normalized_model       # noqa: E402
from reference_utils import compute_path_spline                # noqa: E402
from eval_b1 import _kappa_fn, estimate_dt                     # noqa: E402

MODEL = os.path.join(os.path.dirname(HERE), "mpc", "residual_model_nomass_s0.pt")
OUT   = os.path.join(HERE, "matlab", "fig_openloop.mat")
HORIZONS = [1, 5, 10, 20]                 # 스텝 (dt=0.1 -> 0.1 / 0.5 / 1.0 / 2.0 s)
STRIDE   = 5                              # 시작점 간격
CH = ["ds", "dn", "dalpha", "dv"]

B_DEPLOY = np.zeros((6, 4)); B_DEPLOY[1, 1] = 1; B_DEPLOY[2, 2] = 1
B_ALL    = np.zeros((6, 4))
for i in range(4):
    B_ALL[i, i] = 1


def highway_kappa():
    pi = compute_path_spline(np.load(os.path.join(HERE, "highway", "highway_waypoints.npy")))
    sp = make_interp_spline(pi["dense_s"], pi["kappa"], k=3)
    lo, hi = pi["dense_s"][0], pi["dense_s"][-1]
    return lambda s: sp(np.clip(s, lo, hi))


def lap1(path):
    a = np.load(path)
    w = np.where(np.diff(a[:, 0]) < -100)[0]      # 랩 경계에서 s 가 되감긴다
    return a[:w[0] + 1] if len(w) else a


def rollout(a, dt, kf, N, model, B):
    """N 스텝 개루프 적분 후 실제 상태와의 오차. 반환 (4,) 채널별 RMS."""
    s, n, al, v, D, de = a.T
    u = np.stack([np.gradient(D, dt), np.gradient(de, dt)], 1)
    idx = np.arange(0, len(a) - N - 1, STRIDE)
    err = np.zeros((len(idx), 4))
    for j, i in enumerate(idx):
        x = a[i].copy()
        for k in range(N):
            x = predict_next_state(x, u[i + k], kf, dt)
            if model is not None:
                f = np.array([[x[1], x[2], x[3], x[4], x[5], float(kf(x[0]))]],
                             dtype=np.float32)
                with torch.no_grad():
                    x = x + B @ model(torch.from_numpy(f)).numpy()[0]
        err[j] = x[:4] - a[i + N][:4]
    return np.sqrt((err ** 2).mean(0))


def main():
    model = load_normalized_model(MODEL, device="cpu")
    kf_h = _kappa_fn()
    kf_w = highway_kappa()

    # (이름, 궤적, kappa, 학습분포 안/밖)
    CASES = [
        ("Hockenheim v=12",  "b1/v12_nom.npy",       kf_h, "in"),
        ("Hockenheim v=14",  "b1/v14_nom.npy",       kf_h, "out (speed)"),
        ("Highway No.1",     "highway/hw_nom.npy",   kf_w, "out (track)"),
        ("Low friction 0.3", "mu03/mu03_v9_nom.npy", kf_h, "out (friction)"),
    ]
    MODES = [("nominal", None, None),
             ("+residual (deployed)", model, B_DEPLOY),
             ("+residual (all ch.)",  model, B_ALL)]

    nC, nH, nM = len(CASES), len(HORIZONS), len(MODES)
    E = np.zeros((nC, nH, nM, 4))
    dts = np.zeros(nC)

    print("개루프 예측 오차 RMS — 제어기 없이 모델만으로 N 스텝 적분")
    print(f"모델: {os.path.basename(MODEL)}  (학습 = Hockenheim, 속도 최대 12.23 m/s)\n")
    for ci, (name, f, kf, dist) in enumerate(CASES):
        a = lap1(os.path.join(HERE, f))
        dt = estimate_dt(a[:, 0], a[:, 3]); dts[ci] = dt
        print(f"── {name}   [{dist}]   dt={dt:.3f}s, {len(a)} steps")
        print(f"{'horizon':>9}{'model':>22}{'Δs(m)':>9}{'Δn(m)':>9}"
              f"{'Δα(rad)':>10}{'Δv(m/s)':>10}")
        for hi, N in enumerate(HORIZONS):
            for mi, (lbl, mdl, B) in enumerate(MODES):
                e = rollout(a, dt, kf, N, mdl, B)
                E[ci, hi, mi] = e
                print(f"{N*dt:8.1f}s{lbl:>22}{e[0]:9.4f}{e[1]:9.4f}"
                      f"{e[2]:10.5f}{e[3]:10.4f}")
            r = 100 * (1 - E[ci, hi, 1, 1] / E[ci, hi, 0, 1])
            print(f"{'':>31}Δn {r:+.0f}%")
        print()

    print("=" * 72)
    print("Δn 예측오차 개선율 — 예측 구간별 (배포 설정). 양수 = 잔차가 모델을 개선")
    print("=" * 72)
    print(f"{'조건':>18}{'구분':>16}" + "".join(
        f"{h*dts[0]:7.1f}s" for h in HORIZONS))
    IMP = np.zeros((nC, nH))
    for ci, (name, _, _, dist) in enumerate(CASES):
        for hi in range(nH):
            IMP[ci, hi] = 100 * (1 - E[ci, hi, 1, 1] / E[ci, hi, 0, 1])
        print(f"{name:>18}{dist:>16}" + "".join(f"{v:7.0f}%" for v in IMP[ci]))
    print()
    print("  학습 분포 안에서는 2 s 예측 횡오차가 89% 줄지만, 분포를 벗어나면")
    print("  단조롭게 무너져 미학습 트랙과 저마찰에서는 오히려 나빠진다.")
    print("  즉 그 두 조건의 폐루프 이득은 '모델이 정확해져서'가 아니다.")
    print("  (results/b1/ 의 가중치 스윕에서 Q_n 만 올린 nominal 이 잔차를 따라잡는")
    print("   현상과 같은 방향의 증거다 — 그 영역에서 잔차는 실효 게인처럼 작용한다.)")

    savemat(OUT, dict(
        # 필드명 case 는 MATLAB 예약어(switch/case)라 case_ 로 둔다
        case_     = np.array([c[0] for c in CASES], dtype=object),
        dist      = np.array([c[3] for c in CASES], dtype=object),
        mode      = np.array([m[0] for m in MODES], dtype=object),
        channel   = np.array(CH, dtype=object),
        horizon_s = np.array(HORIZONS) * dts[0],
        horizon_n = np.array(HORIZONS),
        dt        = dts,
        err       = E,          # (case, horizon, mode, channel)
        improve_dn = IMP,       # (case, horizon) Δn 개선율 [%], 배포 설정
    ))
    print(f"\n저장: {OUT}")


if __name__ == "__main__":
    main()
