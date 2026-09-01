# 미학습 트랙 일반화 (USA Highway No.1), 2026-09-01

## 왜 했나

일반화 축이 '적재' 하나뿐이었다. **트랙 축**을 추가했다.

## 설정

| 항목 | 값 |
|---|---|
| 학습 | **Hockenheim** (`mpc/residual_model_nomass_s0.pt`) — **재학습 없음** |
| 시험 | **USA Highway No.1** (`USA_HighwayNo1_3DMapping.rd5`, TestRun `highway_no1`) |
| 경로 | `highway_waypoints.npy` — IPGDriver 주행을 `scripts/record_path.py` 로 녹화 (461점, 931 m) |
| 속도 / 적재 / 가중치 | 12 m/s / 48 t / Q_n = 1e-4, 잔차 scale 0.3 |

**곡률 검증:** 이 경로의 κ 는 학습 데이터 분포(1~99%) 밖 비율이 **0.0%** 다.
외삽이 아니라 순수한 경로 일반화다. 최소 회전반경 Hockenheim 37.8 m vs Highway 91.8 m.

## 결과

| | 완주 | 거리 | 평균 \|n\| | 최대 \|n\| | 여유 대비 | 채터 |
|---|---|---|---|---|---|---|
| nominal | **X** | 624 m | 0.196 | **0.835** | **165%** | 0.136 |
| **residual** | **O** | **925 m** | **0.067** | **0.213** | **42%** | 0.072 |
| | | | −66% | −75% | | −47% |

## 왜 결정적인가 — 성능이 아니라 가능/불가능이 갈렸다

이 도로는 **차선 3.5 m, 트럭 2.49 m** 라 좌우 여유가 각 **0.5 m** 뿐이다.

```
nominal  최대 0.835 m  ->  여유의 165%  ->  바퀴가 도로 밖
residual 최대 0.213 m  ->  여유의  42%  ->  완주
```

TruckMaker 로그:
```
ERROR      Vehicle leaves road at about x=-5.5682, y=223.921 TireNo=1
SIM_ABORT  highway_no1  70.425s  624.986m
```

**nominal 은 이 도로를 주행할 수 없고 residual 은 할 수 있다.** Hockenheim(서킷,
폭이 넉넉)에서는 둘 다 완주했으므로 드러나지 않던 차이다.

## 의미

잔차가 특정 경로의 곡률 프로파일을 암기한 것이 아니라 **차량 자체의 모델 오차를
학습**했음을 보여준다. 곡률 분포가 다른(반경 2.4배) 다른 종류의 도로(서킷 → 측정
고속도로)에서 재학습 없이 그대로 작동했다.

## 파일

| 파일 | 내용 |
|---|---|
| `hw_nom.npy` / `hw_res.npy` | 주행 궤적 `[s, n, alpha, v, D, delta]`, dt=0.1 |
| `highway_waypoints.npy` | 녹화한 기준 경로 (461 × 2, x/y) |

## 재현

```bash
python3 results/highway_summary.py      # 표 + results/matlab/fig_highway{,_traj}.mat
```

MATLAB: `results/matlab/plot_highway.m`

## 미실시

- 다른 속도(10, 14 m/s)에서의 반복
- 다른 적재에서의 반복 (48 t 만 수행)
- 반복 주행 (각 조건 1회씩이라 편차 미측정)
