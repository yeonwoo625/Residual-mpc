# R 벌점↑ + first-order 실험 (마지막 남은 실차 레버)

**작성 2026-07-09.** 목적: 조향 변화율 벌점 R_DELTA를 키우면 first-order(값+미분) MPC의
과조향 진동이 억제되어 **완주**하는지, 그리고 완주한다면 그게 **미분을 유익하게 쓴 것인지
(FF보다 나음)** 아니면 **조향을 죽여 feedforward에 수렴한 것인지**를 판별한다.

R_DELTA는 이미 env로 조절 가능 (`mpc/acados_settings.py`: `R[1,1]=R_DELTA`, 기본 2e-4).

---

## 0. 핵심 판별 (반증가능) — "완주"만으로는 부족

R_DELTA↑는 **명목·잔차 조향 둘 다** 매끄럽게 만든다(선택적으로 잔차 과조향만 잡는 게 아님).
그래서 완주해도 두 가지 경우가 있다:

- **Case A — first-order 구제 성공:** 어떤 R*에서 완주 + **|n| 추종이 FF보다 좋음**(미분이
  언더스티어를 실제로 보상) → first-order가 R 튜닝으로 살아남. ★긍정 기여.
- **Case B — feedforward化:** 완주하지만 그 R*에서 **조향이 억눌려 |n|이 FF와 비슷하거나 나쁨**
  + 조향 활동이 명목 수준으로 죽음 → R이 그냥 미분 효과를 지운 것 → **FF가 더 낫다** 확증.

→ **반드시 같은 R*에서 first-order vs FF를 공정 비교**해야 결론이 난다.

---

## 1. 실행 방법 (λ-스윕 때와 동일, R_DELTA만 추가)

first-order는 이전 λ-스윕처럼 `RESIDUAL_JAC_SCALE=1` + `RESIDUAL_DBG_LOG`로 구동
(실차 구동 노드 = `mpc_ros_node.py` → `MPCSolverResidual`).

**공통 env (구동 셸에 설정 — λ-스윕 돌릴 때 쓰던 방식 그대로):**
```bash
export RESIDUAL_MODEL_PATH=/opt/mpc/residual_model.pt   # 배포 8-dim 모델 (기존과 동일)
export RESIDUAL_JAC_SCALE=1        # full first-order (값+미분)
export RESIDUAL_FF=0
unset  RESIDUAL_MASK_JAC
```

각 실행마다 **R_DELTA와 로그 경로만** 바꾼다:
```bash
export R_DELTA=<값>
export RESIDUAL_DBG_LOG=/home/vilab/CarMaker/mpc_host/logs/R_<값>.npy
# → mpc_ros_node.py 구동 (λ-스윕 때와 같은 절차) + CarMaker Hockenheim Start
```

---

## 2. 스윕 순서 (완주할 때까지 R_DELTA를 ×3~×10씩)

기본 2e-4에서 first-order는 발산(s≈280 onset). 아래를 순서대로:

| 순서 | R_DELTA | 로그 파일 |
|---|---|---|
| 1 | `1e-3` (×5) | logs/R_1e-3.npy |
| 2 | `3e-3` (×15) | logs/R_3e-3.npy |
| 3 | `1e-2` (×50) | logs/R_1e-2.npy |
| 4 | `3e-2` (×150) | logs/R_3e-2.npy |
| 5 | `1e-1` (×500) | logs/R_1e-1.npy |

- **완주하는 첫 값 = R\***. 거기서 멈춤(더 키우면 조향만 더 죽음).
- 완주 = s가 ~2540까지 도달(랩 완주). 발산 = |n|>2 이후 이탈.
- 발산해도 위험하면 즉시 Ctrl-C. (실차 안전: |n| 커지면 중단)

---

## 3. 공정 비교 — R*에서 FF도 실행

R*를 찾으면, **같은 R*에서 feedforward**도 한 번:
```bash
export RESIDUAL_JAC_SCALE=0 RESIDUAL_FF=1
export R_DELTA=<R*>
export RESIDUAL_DBG_LOG=/home/vilab/CarMaker/mpc_host/logs/ffR_<R*>.npy
# 구동 + Hockenheim
```
(이미 있는 기본-R FF 로그 `logs/ff.npy`는 R=2e-4라 직접 비교 불가 → 같은 R*로 새로 받아야 공정)

---

## 4. 분석 (완주·추종·조향활동 비교)

수집 끝나면:
```bash
python3 /home/vilab/CarMaker/mpc_host/analyze_R_sweep.py \
    /home/vilab/CarMaker/mpc_host/logs/R_*.npy \
    /home/vilab/CarMaker/mpc_host/logs/ffR_*.npy
```
출력에서 볼 것:
- **completed?** — 랩 완주 여부
- **|n| max / p95 / mean** — 추종 정확도 (작을수록 좋음)
- **steer |Δδ| mean/max [deg]** — 조향 활동 (명목 수준으로 죽었으면 feedforward化)

**판정:**
- first-order@R* **|n| < FF@R* |n|** → **Case A (구제 성공)** ★
- first-order@R* **|n| ≥ FF@R* |n|** 이고 조향 억눌림 → **Case B (feedforward化, FF가 답)**

---

## 5. 정직한 예상 & 캐비아트

- **예상은 Case B.** R_DELTA↑는 잔차 과조향만이 아니라 **모든 조향을 억제**하므로,
  완주하더라도 미분의 언더스티어 보상 효과까지 함께 눌려 FF 대비 이득이 없을 공산이 큼.
  (인과 미분 −0.51이 명목을 상쇄하는 근본 물리는 R로 사라지지 않음 — R은 증상만 억제)
- **하지만 Case A면 대박:** first-order가 R 튜닝으로 살고 FF보다 추종이 좋으면,
  "미분은 못 쓴다"가 뒤집힘. 그래서 **한 번은 해볼 가치**가 있음.
- 어느 쪽이든 결론이 강해짐: A면 긍정 기여, B면 "값만"의 근거를 **제어기 튜닝까지 소거**해 완성.

## 6. 안전 (공유 PC)
- 종료는 Ctrl-C (kill 금지). ROS_DOMAIN_ID=25. CMROS2/TMROS2 수정 금지.
- 발산 조짐(|n| 급증) 시 즉시 중단.
