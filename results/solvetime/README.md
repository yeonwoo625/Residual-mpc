# MPC 연산시간 (2026-09-02)

## 왜 했나

"nominal 과 residual 의 MPC 연산시간을 비교해 달라"에서 출발했으나, 계측해 보니
residual 이 **10 Hz 예산을 넘고 있었다.** 원인을 추적한 결과 알고리즘 비용이 아니라
반복 횟수 설정 문제였고, 고쳐서 4.5배 줄였다.

## 무엇이었나

l4acados `ResidualLearningMPC.solve()` 는 `nlp_solver_max_iter` 만큼
preparation/feedback 을 **무조건** 반복한다 — 수렴 시 빠져나오는 분기는
`rti_log_residuals` 가 켜져 있을 때만 존재한다. acados 기본값 100 이 그대로
적용되어 **제어 1스텝마다 SQP 를 100회** 돌고 있었다. nominal 은 acados 자체
SQP_RTI 로 1회만 돈다 — 속도 문제이자 **계산량 불일치**였다.

offline 으로 쪼개 보면 SQP 1회는 0.92 ms 다 (잔차 신경망 0.28, 공칭 적분기 0.13,
QP 0.15, 파라미터 set/get 0.31). **계산 자체는 문제가 아니었다.**

반복 횟수를 훑으면 **10회에서 이미 수렴**해 100회 해와 조향 명령이 0.002° 이내로
같다. 나머지 90회는 수렴한 문제를 다시 푸는 순수 낭비다.

## 결과 (주행 중 계측, LOG_SOLVETIME)

| 구성 | 중앙값 | p95 | 최대 | n | 예산 대비 |
|---|---|---|---|---|---|
| nominal | 3.51 | 4.84 | 25.8 | 235 | 4% |
| residual (SQP 100, 수정 전) | **109.3** | 121.9 | 129.5 | 135 | **89% 의 스텝에서 초과** |
| residual (SQP 10, 수정 후) | **24.2** | 32.2 | 43.4 | 446 | **24%** |

두 점(10회 24.2, 100회 109.3)을 맞추면 반복 1회당 **0.95 ms** — offline 에서 잰
0.92 ms 와 일치한다. 진단이 맞았음을 주행 데이터가 확인해 준다.

## 파일

| 파일 | 내용 |
|---|---|
| `nominal.npy` | nominal, 스텝별 solve 시간 [ms] (235) |
| `residual_sqp100.npy` | residual 수정 전 (135) |
| `residual_sqp10.npy` | residual 수정 후 (446) |
| `traj_res_sqp10.npy` | 수정 후 궤적 6열 `[s,n,alpha,v,D,delta]`, dt=0.1 s |

전부 Hockenheim, v=10, 48 t, `Q_n=1e-4`, `DDELTA_MAX=0.262`(15 °/s),
`RESIDUAL_FF=1`, `RESIDUAL_SCALE=0.3`, N=20, Tf=2.0 s, HPIPM, use_cython=True,
`OMP_NUM_THREADS=1`.

## 한계 — 정직하게

**폐루프 궤적은 포개지지 않는다.** 겹치는 구간(s = 20~395 m)에서 수정 전
(`../ddelta/v10_res.npy`) 대비:

| | SQP 10 | SQP 100 |
|---|---|---|
| mean \|n\| | 0.068 | 0.073 |
| max \|n\| | 0.255 | 0.305 |

집계값은 같거나 약간 낫지만 점별 차이는 rms 0.108 m 다. 이유가 둘 있다.
① 수정 전에는 solve 가 109 ms 로 제어 주기 100 ms 를 89% 의 스텝에서 넘겨
**제어가 늦게 나가고 있었다** — 지금은 24 ms 라 제때 나간다. 폐루프 타이밍이
달라졌으니 궤적이 같을 수 없다(그리고 이건 개선이다).
② 각 조건 1회 주행이라 **주행 간 편차를 측정하지 않았다.** 위 차이가 SQP 변경
때문인지 편차인지 이 데이터로는 구분되지 않는다.

offline 검증(0.002° 일치)이 보장하는 것은 **같은 상태를 주면 같은 해가 나온다**
까지다. 폐루프 375 m 누적 차이는 그와 별개 문제다.

## 재현

```bash
# 주행 계측
export USE_RESIDUAL=1 RESIDUAL_FF=1 RESIDUAL_SCALE=0.3
export REFERENCE_PATH=/home/vilab/CarMaker/mpc_host/hockenheim_waypoints_1lap.npy
export TARGET_SPEED=10 DDELTA_MAX=0.262
export LOG_SOLVETIME=/tmp/solvetime.npy
./scripts/1_server.sh                       # 주행 후 Ctrl-C 시 통계 출력 + 저장
SQP_ITER=100 ./scripts/1_server.sh          # 수정 전 재현

# offline 구간분해 + 반복 스윕 (사용법은 파일 상단 주석)
python3 results/prof_residual_sqp.py

python3 results/solvetime_summary.py        # 표 + results/matlab/fig_solvetime.mat
```

MATLAB: `results/matlab/plot_solvetime.m`

## 미실시

- `SQP_ITER=1` 주행 검증 — offline 0.99 ms 로 nominal 과 계산량이 같아지나
  해가 달라진다(같은 상태에서 조향 최대 11.5° 차이). 주장에 쓰려면 재검증 필요
- 각 구성 반복 주행 (1회씩이라 주행 간 편차 미측정)
- nominal 의 in-loop 3.5 ms 와 offline 0.6 ms 의 차이(소켓·Frenet 변환·CPU 경합)
  분해 — residual 비교에는 영향이 없어 넘어갔다
