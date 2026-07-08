# 독립-조향-여기 실험 설계 (ⓑ: first-order를 살릴 수 있는 유일한 원리적 길)

**작성 2026-07-09.** 목적: 닫힌 루프 데이터가 주는 조향 미분 ∂g/∂δ ≈ −0.54가
**진짜 인과(물리)인지, 아니면 상관(교란) 아티팩트인지**를 판별하고, 후자면
first-order MPC를 완주시킬 수 있는지 검증한다.

---

## 0. 가설 (반증가능)

닫힌 루프에서 δ는 경로오차 상태(n, α, κ)와 강하게 공선(共線). 그래서 학습된
∂g/∂δ = −0.54는 "δ를 바꾸면 잔차가 어떻게 되나"(인과)가 아니라 "δ가 클 때 잔차도
크더라"(상관)일 수 있다. **δ를 상태와 무관하게 독립적으로 흔들어(excitation)** 공선을
깨면, 진짜 인과 ∂g/∂δ를 식별할 수 있다.

**결정 분기 (어느 쪽이든 논문 기여가 됨):**
- **Case 1 — 상관 아티팩트:** 인과 ∂g/∂δ ≈ 0 또는 명목과 같은 부호(+)
  → −0.54는 교란이었음 → 재학습한 모델로 **first-order 완주 가능** → first-order 살릴 수 있음.
- **Case 2 — 진짜 물리:** 인과 ∂g/∂δ ≈ −0.5 (여전히 명목 상쇄)
  → 언더스티어가 실제로 조향 효과를 깎음 → first-order는 **본질적으로 과조향** →
  데이터로 못 고침 → **구조적 불안정을 인과 수준에서 확증**(가장 강한 결론).

---

## 1. 왜 지금 데이터로는 안 되나 (한 줄)

닫힌 루프: δ = δ_MPC(n,α,κ,...) → VIF(δ) 매우 큼, δ⊥(상태에 직교한 δ 성분) std ≈ 0
→ ∂g/∂δ 식별 불가(교란값만 나옴). **δ에 외생 변동을 주입해야 식별 가능.**

---

## 2. Excitation 설계

수집기(`mpc/mpc_data_collector.py`)에 이미 OU 조향 여기가 있음:
`exc_de = θ·exc_de + N(0, EXCITE_DELTA)`, `delta = clip(δ_MPC + sc·exc_de)`,
`sc = (1 − |n|/EXC_N_LIMIT)·min(1, EXC_VREF/v)`.

**주의 — 기본값은 고속에서 프로브를 거의 죽인다:**
- `EXC_VREF=3` → v=12에서 sc ×= 3/12 = **0.25** (프로브 75% 감쇠)
- `EXC_N_LIMIT=1` → |n|=0.4에서 sc ×= 0.6 (또 감쇠)

식별에는 sc ≈ 1이 필요. 그래서 **VREF·N_LIMIT를 크게** 풀되 진폭은 안전하게:

| 파라미터 | 권장값 | 이유 |
|---|---|---|
| `EXCITE_DELTA` | **0.02 rad**(~1.1°) 시작 → 안전하면 0.03 | 프로브 진폭 |
| `EXC_THETA` | **0.8** (시상수 ~0.5s) | 너무 높으면 MPC가 되받아 상쇄→재공선; 낮으면 액추에이터 거침 |
| `EXC_N_LIMIT` | **4.0** | sc가 코너에서도 ≈1 유지 (안전 easing은 최소만) |
| `EXC_VREF` | **12.0** | v=12에서 속도감쇠 ≈1 |
| `EXCITE_D` | **0.0** | 조향만 여기 (종방향은 무관) |

OU 정상상태 std ≈ EXCITE_DELTA/√(1−θ²) = 0.02/0.6 ≈ 0.033 rad, sc≈0.9 →
**적용 δ⊥ RMS ≈ 0.03 rad(~1.7°)** 목표. δ_MPC의 코너 변동과 맞먹어 공선을 깰 수 있음.

---

## 3. 수집 프로토콜

```bash
# 환경 초기화 (잔차/FF 흔적 제거)
unset USE_RESIDUAL RESIDUAL_FF RESIDUAL_JAC_SCALE RESIDUAL_DBG_LOG RESIDUAL_MODEL_PATH

# --- 1차: 안전 확인용 저속 (8 m/s) ---
export REFERENCE_PATH=/home/vilab/CarMaker/mpc_host/hockenheim_waypoints_1lap.npy
export TARGET_SPEED=8
export EXCITE_DELTA=0.02 EXC_THETA=0.8 EXC_N_LIMIT=4.0 EXC_VREF=12.0 EXCITE_D=0.0
export OUT_NPZ=/home/vilab/CarMaker/mpc_host/excite_probe8.npz
python3 mpc/mpc_data_collector.py     # 명목 MPC + 조향여기로 주행. |n|<0.5 유지 확인 후 Ctrl-C
```
- **안전 게이트:** 콘솔 `s=.. v=..`와 실차 |n| 관찰. |n|이 0.6을 넘으면 즉시 Ctrl-C,
  EXCITE_DELTA를 0.015로 낮춰 재시작. Ctrl-C로 정상 종료해야 save() 실행됨.
- 1랩(~2.5km) 완주 후 계속 돌려 **N ≥ 4000~5000** 누적 (append 됨).

```bash
# --- 2차: 운영 속도 (12 m/s), 안전 확인되면 ---
export TARGET_SPEED=12
export OUT_NPZ=/home/vilab/CarMaker/mpc_host/excite_probe12.npz
python3 mpc/mpc_data_collector.py
```
- 코너에서 여기가 위험하면 EXCITE_DELTA를 0.015로. **여러 랩** 돌려 곡률 범위 커버.

---

## 4. 탈공선 검증 (수집 직후)

```bash
python3 /home/vilab/CarMaker/mpc_host/identify_causal_jac.py \
    /home/vilab/CarMaker/mpc_host/excite_probe12.npz \
    /home/vilab/CarMaker/mpc_host/hockenheim_nomass.npz   # 2번째 = 닫힌루프 비교군
```
**통과 기준 (excited가 closed-loop보다):**
- VIF(δ) : 닫힌루프 큰 값(수십) → excited **< 5** 이어야 식별 신뢰.
- δ⊥ std : 닫힌루프 ≈0 → excited **≥ 0.02 rad** (독립 변동 확보).
- corr(δ⊥, n)·corr(δ⊥, κ) ≈ 0.

미달이면 진폭↑ 또는 데이터↑ 후 재수집. (억지로 진행 금지 — 식별 안 됨)

---

## 5. 인과 미분 식별 (모델-무관, 닫힌형)

`identify_causal_jac.py`가 **Frisch–Waugh 부분회귀**로 계산 (MLP 안 씀 → "잘못 만들었다"
반박 불가):
1. δ(=col4)를 나머지 상태[n,α,v,D,κ]에 회귀 → 잔차 δ⊥ (외생 성분).
2. 타깃 Δn을 같은 상태에 회귀 → 잔차 Δn⊥.
3. **인과 ∂(Δn)/∂δ = slope(Δn⊥ ~ δ⊥)**, 표준오차·R² 동반.
4. 같은 계산을 **닫힌루프 데이터**에도 → 교란값(−0.54류) 재현 후 **나란히 비교**.
5. (선택) excited 데이터로 MLP 재학습 → autodiff ∂g/∂δ 로 교차확인.

---

## 6. 결정 분기 및 최종 실차 검증

```
인과 ∂(Δn)/∂δ 측정
├─ |인과| 작음(≈0) or 부호 + (Case 1)
│    → excited 데이터로 residual 모델 재학습
│    → RESIDUAL_JAC_SCALE=1 (first-order) 실차 주행
│    → 완주 + solve time 측정
│         ├─ 완주 O → first-order 살릴 수 있음(능동여기 필요) ★기여
│         └─ 완주 X → 미분 외 다른 요인, 추가 진단
└─ 인과 ≈ −0.5 (Case 2)
     → 명목 상쇄가 진짜 물리 → first-order 본질적 과조향
     → 구조적 불안정을 인과 수준에서 확증 ★더 강한 기여 (실차 재시도 불필요)
```

---

## 7. 정직한 보고 (어느 결과든 기여)

- **Case 1:** "first-order 발산은 데이터의 공선에 의한 교란 미분 탓이며, **능동 조향여기로
  탈공선**하면 인과 미분(≈0)을 식별해 first-order를 완주시킬 수 있다." → 방법론적 기여.
- **Case 2:** "탈공선 후에도 인과 미분이 명목을 상쇄 → first-order 과조향은 **교란이 아니라
  실제 차량 물리**. 따라서 residual은 값만 쓰는 zero-order가 원리적으로 옳다." → 구조 확증.

두 경우 모두 **"미분을 안 쓴다"는 선택을 인과 수준에서 정당화**한다.

## 8. 안전·공유PC 수칙 (재확인)
- Ctrl-C로 정상 종료(atexit/save 보장). kill -9 금지.
- ROS_DOMAIN_ID=25 유지. CMROS2/TMROS2 수정 금지.
- OUT_NPZ는 개인 경로(mpc_host) 사용.
