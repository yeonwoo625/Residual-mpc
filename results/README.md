# 결과 데이터 (Hockenheim, 10 m/s)

payload-conditioned residual MPC(feedforward) vs nominal MPC의 폐루프 주행 궤적.

## 궤적 파일 (`traj_{nom,ff}_{32,48,56}.npy`)
각 파일은 `(N, 6)` numpy 배열, 컬럼:

| idx | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| 의미 | s (경로거리) | n (횡오차) | alpha (횡각) | v (속도) | D (스로틀) | delta (조향) |

- `nom` = nominal MPC (USE_RESIDUAL=0)
- `ff`  = feedforward residual MPC (RESIDUAL_FF=1, RESIDUAL_SCALE=0.3, MASS 조건화)
- `32/48/56` = 총 적재질량 [t] (공차 32t + 적재 0/16/24t @ CoG x=4.33)

## 성능 요약 (|n| = 횡추종오차)
| 무게 | Nominal 평균 | FF 평균 | 개선 | 급코너 개선 |
|---:|---:|---:|---:|---:|
| 32t | 0.303 | 0.096 | −68% | −62% |
| 48t | 0.325 | 0.109 | −66% | −58% |
| 56t | 0.329 | 0.105 | −68% | −54% |

세 무게 모두 완주. 저~고적재 전 범위에서 −66~68% 개선.

## 그림 (`figures/`)
- `fig1_trajectory_error.png` — 트랙 위 궤적을 |n|으로 색칠 (nominal vs FF)
- `fig2_error_vs_s.png` — 경로거리별 |n|
- `fig3_bar_comparison.png` — 평균/RMS/급코너 막대
- `fig4_mass_dependence.png` — 무게→Δn 잔차 (corr +0.99, 조건화 동기)
- `fig5_steering_win.png` — 조향 명령 비교
- `fig6_why_feedforward.png` — full-Jacobian 발산 vs feedforward
- `fig7_mass_vs_throttle_error.png` — 무게→스로틀/오차
- `fig8_multiload.png` — 32/48/56t 성능표 (핵심 결과)

재현: `docs/payload_residual_mpc.md` 참고.
