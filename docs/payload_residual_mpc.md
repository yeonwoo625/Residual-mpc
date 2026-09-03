# 적재 조건화 Residual MPC — 진행 정리

TruckMaker(8x4 Actros) Frenet kinematic MPC + 학습 잔차(residual)로, **적재함 무게·무게중심을
입력으로 받는** residual MPC를 구현하고 nominal 대비 성능을 검증한 기록.

## 1. 목표
1. 잔차 MPC가 nominal MPC보다 추종 성능이 좋을 것
2. 잔차 MLP가 **적재 무게(payload)** 를 입력으로 받을 것 (조건화)

## 2. 구조
- 잔차 모델: `ConditionedResidualModel` (concat). 입력 `[n,alpha,v,D,delta,kappa, mass, cog]`(8), 출력 `[Δs,Δn,Δα,Δv]`.
- 런타임 배선:
  - `kappa_feature_selector.py`: 환경변수 `MASS,COG_X`를 특징 뒤에 붙여 8차원 생성.
  - `residual_model_wrapper.py`: config의 `context_dim>0`이면 조건화 모델 로드.
  - `mpc_solver_residual.py`: 조건화 모델인데 MASS/COG_X 없으면 명확히 에러(가드).
  - 서버는 `mpc/residual_model.pt`를 로드(주의: train은 `models/`에 저장 → 복사 필요).

## 3. 핵심 발견
### (a) 정속 주행에선 조건화가 잉여
완만맵 4 m/s, 무게 4종(32/40/48/56 t). Δv는 무게 의존이나, **정속 유지 시 스로틀 D가 이미
무게를 인코딩**(corr(D,mass)=+0.47) → 상태만으로 예측 가능. 조건화 ablation 동률, held-out
48 t에선 조건화가 오히려 나쁨(과적합). 횡방향(Δn)은 kinematic이라 무게 무관(≈0).

### (b) 급코너(동역학 영역)에선 횡 잔차가 무게 의존
Hockenheim(R_min 38 m) 12 m/s. 급코너(a_y>2)에서 Δn RMS가 무게 따라 단조 증가:
32k 0.090 → 56k 0.110, **corr(mass, Δn)=+0.992**. 조향 δ는 무게를 모르는 kinematic이 정하므로
무게가 제어에 안 새어듦 → 여기선 조건화가 물리적으로 의미 있음.
ㅆ
### (c) 순진한 잔차 주입은 불안정
잔차의 거친 Jacobian이 조향↔편차 되먹임 루프를 만들어 진동(조향 스텝변화 nominal의 30배) →
이탈. scale·jac_reg 튜닝으로도 못 잡음.

### (d) 피드포워드 잔차로 해결 → nominal 격파
`FeedforwardResidualModel`: 잔차 **값은 쓰되 Jacobian=0**으로 반환 → 잔차를 "고정 외란"으로
취급, 되먹임 진동 제거. (`RESIDUAL_FF=1`)

## 4. 최종 결과 (Hockenheim, 48 t, 10 m/s, 둘 다 완주 s=0~2540 m)
| 지표 | Nominal | 잔차 MPC (FF, 무게입력, scale 0.3) | 개선 |
|---|---:|---:|---:|
| \|n\| 평균 (전체) | 0.325 | **0.109** | **−66%** |
| \|n\| RMS (전체) | 0.397 | **0.142** | −64% |
| \|n\| 평균 (급코너) | 0.831 | **0.353** | −58% |
| 조향 step-change | 0.07° | **0.24°** | 매끈(발산 없음) |

→ **목표 2개 달성**: 잔차 MPC가 nominal을 크게 개선, MLP는 무게를 입력으로 받음.
- scale 0.5는 과보정으로 조향 떨림(1.05°) 발생 → **scale 0.3에서 떨림 0.24°로 해결 + ?추종도 더 좋아짐**.
- `R_DELTA`(조향율 벌점)는 inter-solve chatter엔 효과 없었음 → scale로 해결.
  > ⚠️ **2026-08-26 정정:** 이 결론은 재검토가 필요하다. 당시 `mpc_solver_residual.py`는
  > 자체 OCP를 만들면서 `R[1,1]=2e-4`를 **하드코딩**하고 있었다(`acados_settings.py`의
  > `R_DELTA` env는 nominal 경로에만 적용). 즉 **잔차 MPC 실행에서는 `R_DELTA`가
  > 애초에 반영되지 않았을 가능성이 높다** — "효과 없음"이 아니라 "적용 안 됨".
  > 현재는 `mpc/cost_weights.py`로 두 경로가 같은 가중치를 쓰므로 재확인 가능.
  > (`docs/R_sweep_experiment_design.md`의 스윕은 실제로 수행되지 않았다 —
  > 로그가 `R_1e-3.npy` 15스텝/1.4 m 하나뿐이다.)

## 5. 재현 레시피
```bash
# best-fit 조건화 모델 학습 (피드포워드는 값 정확도가 중요 → jac_reg 없음)
train_residual.py --data hockenheim_mass.npz --mode concat --split-mode random
cp models/residual_model.pt mpc/residual_model.pt   # 서버 로드 경로

# 실행 (핵심: RESIDUAL_FF=1, MASS/COG_X, scale 0.3, 10 m/s)
RESIDUAL_FF=1 RESIDUAL_SCALE=0.3 USE_RESIDUAL=1 MASS=48000 COG_X=4.330 \
  REFERENCE_PATH=hockenheim_waypoints_1lap.npy TARGET_SPEED=10 \
  LOG_TRAJ=traj_residual.npy  1_server.sh
# nominal 기준선: USE_RESIDUAL=0 ... (같은 조건)
```

## 6. 남은 작업 (논문 완성용)
- (a) 다른 적재(32/40/56 t)에서도 nominal 격파 확인.
- (b) **무조건화(6차원) FF vs 조건화(8차원) FF** 비교 → 무게 입력이 실제로 기여하는지 증명
  ("무게 없이 잔차만으로 이기는 것 아니냐"는 반박 방어).
- 여러 속도(8/10/12)에서 개선 곡선.

## 데이터셋 (mpc_host, repo 미포함)
- `hockenheim_waypoints_1lap.npy` — 기준 경로(2546 m, R_min 38 m)
- `hockenheim_mass.npz` — 무게 4종 × 12 m/s 잔차 데이터(8차원)
- `traj_{nominal,residual}_*.npy` — 폐루프 궤적 로그
