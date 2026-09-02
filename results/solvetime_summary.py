#!/usr/bin/env python3
"""
MPC 연산시간 — nominal vs residual, 그리고 100배 낭비의 발견과 제거.

배경: 주행 중 계측(LOG_SOLVETIME)에서 residual MPC 의 solve 시간이 중앙값
109.3 ms 로, nominal(3.5 ms) 의 31배이자 10 Hz 예산(100 ms)을 넘겼다.
"신경망을 붙였으니 느린 것"으로 넘어갈 수 있었으나, 구간별로 쪼개 보니
계산량 자체는 문제가 아니었다.

  total solve()        88.5 ms   (offline, 동일 조건)
   ├ l4acados prep      0.72 ms  ← 이 중 잔차 신경망 0.28, 적분기 0.13
   └ l4acados feedback  0.15 ms
  합계                  0.87 ms

원인: l4acados 의 solve() 는 nlp_solver_max_iter 만큼 preparation/feedback 을
무조건 반복한다(rti_log_residuals 가 꺼져 있어 조기 종료가 없다). acados 기본값
100 이 그대로 적용되어 제어 1스텝마다 SQP 를 100회 돌고 있었다. 반면 nominal 은
acados 자체 SQP_RTI 로 1회만 돈다 — 속도 문제이자 계산량 불일치였다.

반복 횟수를 훑어 보면 10회에서 이미 수렴해 100회 해와 조향 명령이 0.002° 이내로
같다. 나머지 90회는 수렴한 문제를 다시 푸는 순수 낭비다.

조치: mpc/mpc_solver_residual.py 에 SQP_ITER(기본 10) 추가.
  SQP_ITER=10 -> 88.5 ms 에서 9.2 ms (9.6배), 해는 동일 -> 기존 결과 전부 유효
  SQP_ITER=1  -> 0.99 ms, nominal 과 계산량이 같은 진짜 RTI. 해가 달라지므로
                 주장에 쓰려면 주행 재검증이 필요하다.

계측 조건: N=20, Tf=2.0s, HPIPM PARTIAL_CONDENSING, use_cython=True,
OMP_NUM_THREADS=1, Hockenheim 1랩 경로, residual_model_nomass_s0.pt, RESIDUAL_FF=1.
in-loop 값은 실제 주행 중 mpc_solver_server.py 가 기록한 것이고,
offline 값은 동일 솔버를 소켓/Frenet 변환 없이 반복 호출해 잰 것이다.

재현:  python3 results/prof_residual_sqp.py     (구간분해 + 반복횟수 스윕)
사용:  python3 results/solvetime_summary.py
출력:  표 3개 + results/matlab/fig_solvetime.mat
"""
import os
import numpy as np
from scipy.io import savemat

HERE = os.path.dirname(os.path.abspath(__file__))
MATDIR = os.path.join(HERE, "matlab")

# ---------------------------------------------------------------- 주행 중 계측
# mpc_solver_server.py 의 LOG_SOLVETIME 원본 로그 (Hockenheim, v=10, 48t,
# DDELTA_MAX=0.262, RESIDUAL_FF=1, RESIDUAL_SCALE=0.3, Q_n=1e-4).
INLOOP_FILES = [
    ("nominal",                 "nominal.npy"),
    ("residual (SQP 100, 기존)", "residual_sqp100.npy"),
    ("residual (SQP 10, 수정)",  "residual_sqp10.npy"),
]


def inloop():
    """원본 로그에서 통계를 다시 계산 — 숫자를 코드에 박아 두지 않는다."""
    rows = []
    for nm, fn in INLOOP_FILES:
        a = np.load(os.path.join(HERE, "solvetime", fn))
        rows.append((nm, np.median(a), a.mean(), np.percentile(a, 95),
                     a.max(), len(a)))
    return rows

# ------------------------------------------------------- offline 구간 분해 (median ms)
BREAKDOWN = [
    ("SQP 반복 1회 총합",      0.92),
    ("  l4acados preparation", 0.72),
    ("    - 잔차 신경망",       0.28),
    ("    - 공칭 적분기",       0.13),
    ("    - 파라미터 set/get",  0.31),
    ("  l4acados feedback(QP)", 0.15),
]

# ------------------------------------------- 반복 횟수 스윕 (offline, 100회 해 대비)
#  iters, solve ms, max|dδ| deg, rms|dδ| deg
SWEEP = np.array([
    [  1,  0.99, 11.4592, 3.7210],
    [  2,  1.83, 11.4592, 2.9995],
    [  3,  2.75, 11.4592, 2.5871],
    [  5,  4.59,  7.0575, 1.1163],
    [ 10,  9.20,  0.0024, 0.0008],
    [100, 88.52,  0.0000, 0.0000],
])

NOMINAL_OFFLINE = 0.60          # results/bench_solvetime.py, 동일 조건
BUDGET_MS = 100.0               # 10 Hz 제어 주기


def main():
    IL = inloop()
    print("=" * 66)
    print("1. 주행 중 계측 (LOG_SOLVETIME)")
    print("=" * 66)
    print(f"{'구성':<26}{'중앙값':>8}{'평균':>8}{'p95':>8}{'최대':>8}{'n':>6}")
    for nm, md, mn, p95, mx, n in IL:
        print(f"{nm:<26}{md:>8.2f}{mn:>8.2f}{p95:>8.2f}{mx:>8.1f}{n:>6d}")
    over = 100.0 * (np.load(os.path.join(HERE, "solvetime", "residual_sqp100.npy"))
                    > BUDGET_MS).mean()
    print(f"\n  수정 전: nominal 의 {IL[1][1] / IL[0][1]:.0f}배, "
          f"{BUDGET_MS:.0f} ms 예산을 {over:.0f}% 의 스텝에서 초과")
    print(f"  수정 후: {IL[2][1]:.1f} ms — 예산의 {100 * IL[2][1] / BUDGET_MS:.0f}% "
          f"(p95 {IL[2][3]:.1f} ms)")
    b = (IL[1][1] - IL[2][1]) / 90.0
    print(f"  두 점 맞춤: 반복 1회당 {b:.2f} ms — offline 계측 0.92 ms 와 일치")

    print("\n" + "=" * 66)
    print("2. 어디에 쓰이는가 (offline, SQP 1회 기준)")
    print("=" * 66)
    for nm, ms in BREAKDOWN:
        print(f"{nm:<26}{ms:>8.2f} ms")
    print("\n  -> 실제 계산은 1 ms 미만. 109 ms 는 이것을 100번 반복한 결과였다.")

    print("\n" + "=" * 66)
    print("3. 반복 횟수 스윕 — 10회에서 이미 수렴")
    print("=" * 66)
    print(f"{'SQP 반복':>8}{'solve(ms)':>12}{'max|dδ|(deg)':>15}{'rms|dδ|(deg)':>15}")
    for it, ms, mx, rms in SWEEP:
        print(f"{int(it):>8d}{ms:>12.2f}{mx:>15.4f}{rms:>15.4f}")
    print(f"\n  nominal(offline) = {NOMINAL_OFFLINE:.2f} ms")
    print(f"  SQP 10회: 해는 100회와 동일(0.002°), 속도 {SWEEP[-1,1]/9.20:.1f}배")
    print(f"  SQP  1회: nominal 대비 {SWEEP[0,1]/NOMINAL_OFFLINE:.1f}배 — 잔차의 순수 비용")

    os.makedirs(MATDIR, exist_ok=True)
    out = os.path.join(MATDIR, "fig_solvetime.mat")
    savemat(out, {
        "inloop_names":  np.array([n for n, *_ in IL], dtype=object),
        "inloop_stats":  np.array([[a, b, c, d, e] for _, a, b, c, d, e in IL]),
        "bd_names":      np.array([n for n, _ in BREAKDOWN], dtype=object),
        "bd_ms":         np.array([v for _, v in BREAKDOWN]),
        "sweep":         SWEEP,
        "nominal_offline_ms": NOMINAL_OFFLINE,
        "budget_ms":     BUDGET_MS,
    })
    print(f"\n저장: {out}")


if __name__ == "__main__":
    main()
