"""
MPC 비용 가중치 — 환경변수로 조절 (B1 베이스라인 튜닝 스윕용).

기본값은 기존 하드코딩 값과 **완전히 동일**하다. 환경변수를 아무것도 주지 않으면
이전과 똑같이 동작한다.

  Q_S, Q_N, Q_ALPHA, Q_V, Q_DELTA   상태 가중 (s, n, alpha, v, delta)
  R_D, R_DELTA                      입력 가중 (derD, derDelta = 변화율 벌점)
  QE_S, QE_N, QE_ALPHA              종단 가중. QE_N/QE_ALPHA를 안 주면 Q_N/Q_ALPHA를
                                    따라간다 (기존에도 둘이 같은 값이었다).

주의: acados의 levenberg_marquardt = 1e-3 은 절대값이고 W는 1e-4~1e-2 범위다.
전체 스케일을 크게 바꾸면 정규화 항의 상대적 비중이 달라져 솔버 거동이 변한다.
스윕할 때는 한 축만 바꾸고 전체 스케일은 현 수준으로 유지할 것.

    Q_N=1e-3 ./scripts/1_server.sh      # 횡오차 가중만 10배
"""
import os
import numpy as np


def _f(name, default):
    return float(os.environ.get(name, default))


def get_weights(nu=2, verbose=True):
    """returns (Q, R, Qe) — 기존 코드가 쓰던 것과 같은 형태의 대각 행렬."""
    q_n     = _f("Q_N", 1e-4)
    q_alpha = _f("Q_ALPHA", 1e-5)

    Q = np.diag([
        _f("Q_S", 1e-5),        # s
        q_n,                    # n      ← 횡방향 이탈 (B1 주 튜닝 대상)
        q_alpha,                # alpha  ← 헤딩 오차
        _f("Q_V", 1e-3),        # v
        0.0,                    # D
        _f("Q_DELTA", 0.0),     # delta  ← 조향 '크기' 벌점 (기존 0, 필요시 사용)
    ])

    R = np.eye(nu)
    R[0, 0] = _f("R_D", 2e-4)           # derD     (스로틀 변화율)
    R[1, 1] = _f("R_DELTA", 2e-4)       # derDelta (조향 변화율 = 부드러움)

    Qe = np.diag([
        _f("QE_S", 5e-5),
        _f("QE_N", q_n),                # 기본적으로 Q_N을 따라감
        _f("QE_ALPHA", q_alpha),        # 기본적으로 Q_ALPHA를 따라감
        0.0, 0.0, 0.0,
    ])

    if verbose:
        print(f"[weights] Q_n={Q[1,1]:.2e} Q_alpha={Q[2,2]:.2e} Q_v={Q[3,3]:.2e} "
              f"Q_delta={Q[5,5]:.2e} | R_D={R[0,0]:.2e} R_DELTA={R[1,1]:.2e} "
              f"| Qe_n={Qe[1,1]:.2e}", flush=True)
    return Q, R, Qe
