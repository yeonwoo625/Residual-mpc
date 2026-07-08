"""
탈상관/식별성 진단 — 재수집 데이터가 잔차 미분(∂g/∂delta)을 식별 가능하게
만들었는지 판정. 지표 3종을 구간별(전체/직선/코너/급코너)로 본다.

  (1) delta VIF           : delta가 나머지 입력으로 얼마나 설명되나 (낮을수록 좋음)
  (2) corner sc_n 분포     : dither가 코너서 살아남았나 (중간 점검, 필요조건)
  (3) delta의 κ-잔차 std   : delta에서 κ 성분을 뺀 독립 변동 (진짜 테스트,
                             재수집 후 코너서 '늘어야' delta-κ 잠금이 풀린 것)

사용: python verify_decorrelation.py <before.npz> [after.npz ...] [--nlimit 1.8]
"""
import sys, numpy as np

NAMES = ['n', 'alpha', 'v', 'D', 'delta', 'kappa', 'mass', 'cog']
DI, KI, NI = 4, 5, 0   # delta, kappa, n 열 인덱스

def _vif_delta(X):
    keep = [i for i in range(X.shape[1]) if X[:, i].std() > 1e-9 and i != DI]
    A = np.c_[np.ones(len(X)), X[:, keep]]
    b = np.linalg.lstsq(A, X[:, DI], rcond=None)[0]
    resid = X[:, DI] - A @ b
    r2 = 1 - (resid ** 2).sum() / max(((X[:, DI] - X[:, DI].mean()) ** 2).sum(), 1e-12)
    return r2, 1.0 / max(1e-9, 1 - r2), resid.std()   # δ⊥std = 전체 부분잔차 std

def _kappa_resid_std(X):
    # delta ~ a*kappa + b 회귀 후 잔차 std (delta의 kappa-독립 변동)
    A = np.c_[np.ones(len(X)), X[:, KI]]
    b = np.linalg.lstsq(A, X[:, DI], rcond=None)[0]
    return (X[:, DI] - A @ b).std()

def report(path, nlimit):
    d = np.load(path); X = d['X']; y = d['y']
    k = np.abs(X[:, KI]); nabs = np.abs(X[:, NI])
    segs = [('전체', np.ones(len(X), bool)),
            ('직선  |k|<med', k < np.median(k)),
            ('코너  |k|>med', k >= np.median(k)),
            ('급코너 상위20%', k >= np.quantile(k, 0.8))]
    print(f"\n=== {path}  (N={len(X)}, cols={X.shape[1]}) ===")
    print(f"  {'구간':<14} {'N':>5}  {'δ⊥std':>8}  {'δκ-잔차':>8}  {'VIF(참고)':>9}  {'sc_n':>5}  {'|n|med':>7}")
    for lbl, m in segs:
        r2, vif, dperp = _vif_delta(X[m])
        kr = _kappa_resid_std(X[m])
        scn = np.median(np.clip(1 - nabs[m] / nlimit, 0, 1))
        print(f"  {lbl:<14} {m.sum():>5}  {dperp:>8.4f}  {kr:>8.4f}  {vif:>9.1f}  {scn:>5.2f}  {np.median(nabs[m]):>7.3f}")
    # 잔차 타겟(y) per-channel std — 오염 체크: after가 before 대비 dither 진폭에
    # 비례해 부풀면 명목예측이 잘못된 δ로 계산돼 dither가 타겟에 샌 것. 소폭↑는 정상.
    yn = ['Δs', 'Δn', 'Δα', 'Δv']
    hi = k >= np.quantile(k, 0.8)
    print("  y(잔차타겟) std  전체: " + "  ".join(f"{yn[i]}={y[:, i].std():.4f}" for i in range(y.shape[1])))
    print("               급코너: " + "  ".join(f"{yn[i]}={y[hi, i].std():.4f}" for i in range(y.shape[1])))

if __name__ == "__main__":
    argv = sys.argv[1:]
    nlimit = 1.8
    if '--nlimit' in argv:
        i = argv.index('--nlimit')
        nlimit = float(argv[i + 1])
        del argv[i:i + 2]
    args = [a for a in argv if not a.startswith('--')]
    if not args:
        args = ['/home/vilab/CarMaker/mpc_host/hockenheim_mass.npz']
    print(f"[sc_n은 EXC_N_LIMIT={nlimit} 가정]")
    for p in args:
        report(p, nlimit)
    print("\n판정: 급코너 미식별이 핵심 → 1차 지표 = 급코너 δ⊥std(전체 부분잔차)의")
    print("  절대 증가 (FF+dither OFF baseline 대비). 식별성 ∝ δ⊥std·√N, 비율 VIF 아님.")
    print("  VIF는 R²=1 근처서 불안정하니 참고용. δκ-잔차는 물리(δ-κ) 보조.")
    print("  ★ 최종 판정은 5단계 first-order 폐루프 안정성 — 위는 전부 대리지표.")
