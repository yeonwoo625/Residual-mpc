# Ablation: 무게 입력(조건화) vs 무게 무시 — 무게 채널의 기여

feedforward 잔차 MPC에서 **MLP 입력에 무게(+CoG)를 넣는지 여부**만 다른 두 모델 비교.
차이 = mass 채널만 (split seed=42 고정으로 cond/nomass가 같은 train/val 분할 사용; init RNG는 아키텍처 차이라 3시드로 정량화).

## 파일
`abl_{cond,nomass}_{32,56}[_sN].npy` — 각 `(N,6)` = `[s,n,alpha,v,D,delta]`
- `cond`   = 무게 입력 O (8차원 모델, `MASS`/`COG_X` 조건화)
- `nomass` = 무게 입력 X (6차원 모델)
- seed0 = suffix 없음, seed1/2 = `_s1`/`_s2`
- 48t는 seed0만 (중점 참고). 32t·56t는 seed0/1/2.
- `hockenheim_nomass.npz` = 무조건화 학습 데이터 (mass 열 제거)
- 모델: `mpc/residual_model_{cond,nomass}_s{0,1,2}.pt`

## 결과 (급코너 |n| RMS, 3시드 paired)
| 무게 | 조건화 | 무조건화 | 차이(무조건화−조건화) |
|---:|---:|---:|---:|
| 32t | 0.281±0.028 | 0.332±0.014 | +0.051 |
| 56t | 0.368±0.021 | 0.415±0.050 | +0.047 |

**6페어 (무조건화−조건화)**: +0.030,+0.095,+0.029,+0.073,−0.011,+0.079
- 평균차이 **+0.049 m (~13%)**, SE 0.016
- paired t-test p=0.030, 부호검정 5/6 p=0.109
- → **조건화가 급코너서 ~13% 개선(시사적, modest); n=6라 유의성 경계적. 적재 의존성은 이 데이터로 확인 불가** (1시드의 "적재 따라 증가"는 시드 노이즈였음).

그림: `../figures/fig9_conditioning_ablation.png` (개별 시드 점 표시).
