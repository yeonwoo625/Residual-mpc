# 코드 사용법 (실행·재현 가이드)

> ⚠️ **repo private 유지.** CarMaker/IPG 코드(TMROS2, CMROS2, cmnode_hellocm)는 포함하지 않음. 공유 PC: `ROS_DOMAIN_ID=25`(개인), 종료는 Ctrl-C(kill 금지), CarMaker 라이선스는 다 쓰면 정상 종료.

## 0. 구조 (왜 이렇게 실행하나)

실차 주행은 **서버 + 릴레이 2-프로세스**로 돈다 (구버전 `mpc_ros_node.py`는 미사용):
```
CarMaker(cm_node, DDS) ── /hmg_vehicle_state ──▶ 2_relay.sh (호스트, ROS2, 도메인25)
                          ◀── /hmg_ctrl ────────      │ socket /tmp/mpc_sock
                                                        ▼
                                            1_server.sh (호스트 venv, acados+torch)
                                            = mpc_solver_server.py (MPC 풀이)
```
- 서버는 **호스트 venv**(`mpc_host/venv`, acados/torch/l4acados 있음)에서 소켓으로 대기.
- 릴레이는 **호스트 네이티브 ROS2**라 CarMaker 브리지와 DDS로 직접 통신(컨테이너 DDS 문제 없음).
- 도커(`run_mpc.sh`)는 빌드/테스트용. 실차 주행엔 위 2-프로세스를 쓴다.

---

## 1. CarMaker(TruckMaker) 브리지 띄우기

CarMaker는 **CMRosIF 확장과 함께** 띄워야 `/hmg_vehicle_state`를 발행한다. 일반 실행은 IPGDriver라 MPC가 제어 못 함.
```bash
export ROS_DOMAIN_ID=25
cd /home/vilab/CarMaker/TMROS2
./CMStart.sh                 # TruckMaker + CMExt-CMRosIF.mod
```
확인(새 터미널):
```bash
source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=25
ros2 topic hz /hmg_vehicle_state    # 주행 중 200Hz 나와야 정상
```

---

## 2. 실차 주행 (residual MPC)

**터미널 B — 서버** (env 설정 후 실행, `listening on /tmp/mpc_sock`까지 대기, ~40s 빌드):
```bash
cd /home/vilab/CarMaker/mpc_docker
export USE_RESIDUAL=1 RESIDUAL_FF=1        # RESIDUAL_FF=1 = zero-order(값만, 안정). first-order는 생략
export MASS=48000 COG_X=4.330              # 8-dim 조건화 모델 쓸 때만 필요 (state-only 6-dim이면 불필요)
export REFERENCE_PATH=/home/vilab/CarMaker/mpc_host/hockenheim_waypoints_1lap.npy
export TARGET_SPEED=12
./scripts/1_server.sh
```
**터미널 C — 릴레이** (서버 빌드 끝 + CarMaker Start 후):
```bash
cd /home/vilab/CarMaker/mpc_docker && ./scripts/2_relay.sh    # "relay connected" 뜨면 제어 시작
```
순서: **서버 → CarMaker Start → 릴레이**. 서버·릴레이는 주행 내내 켜두고(Ctrl-C 금지), 완주/발산 후 서버 Ctrl-C.

**주요 residual env (mpc_solver_residual.py):**
| env | 뜻 |
|---|---|
| `USE_RESIDUAL=1` | nominal + 학습 잔차 |
| `RESIDUAL_FF=1` | zero-order(값만, Jacobian=0) — 안정·권장 |
| `RESIDUAL_JAC_SCALE=λ` | first-order, 잔차 Jacobian×λ (1=full, 0=FF). **λ≠0 발산** |
| `RESIDUAL_MODEL_PATH` | 모델 경로 (기본 mpc/residual_model.pt) |
| `MASS`, `COG_X` | 조건화(8-dim) 모델 컨텍스트 |
| `RESIDUAL_DBG_LOG=path` | per-step 12열 로그 저장(발산 진단용) |
| `RESIDUAL_SCALE` | 잔차 출력 배율 (기본 1.0). 기존 실험은 0.3 |
| `RESIDUAL_CLAMP` | 잔차 포화 한계. 값 1개 = 기본값 배율, 4개 = `[Δs,Δn,Δα,Δv]` 직접 지정.<br>기본 `[0.2, 0.1, 0.05, 0.4]` (0.1 s 스텝당). 예: `RESIDUAL_CLAMP=2` |
| `Q_N`, `Q_ALPHA`, `Q_V`, `Q_DELTA` | 비용 가중치 (mpc/cost_weights.py, 기본값=기존 값) |
| `R_D`, `R_DELTA` | 스로틀·조향 rate 벌점 (기본 2e-4). nominal·residual 양쪽에 적용 |

---

## 3. 데이터 수집 (nominal 주행 + 잔차 기록)

`mpc_data_collector.py` = 명목 MPC로 주행하며 (state, 잔차)를 OUT_NPZ에 append.
```bash
unset USE_RESIDUAL RESIDUAL_FF RESIDUAL_JAC_SCALE RESIDUAL_MODEL_PATH   # nominal로
export REFERENCE_PATH=.../hockenheim_waypoints_1lap.npy TARGET_SPEED=12
export OUT_NPZ=.../mydata.npz
# 조건화 라벨 붙이려면: export MASS=48000 COG_X=4.330
python3 mpc/mpc_data_collector.py     # + CarMaker Start(relay 필요) → 주행 → Ctrl-C로 저장
```
**능동 조향여기(excitation, 인과 미분 식별용):** δ를 경로와 무관하게 흔들어 탈공선.
```bash
export EXCITE_DELTA=0.012 EXC_THETA=0.8 EXC_N_LIMIT=2.5 EXC_VREF=12.0 EXCITE_D=0.0
# EXCITE_DELTA=조향 흔들기 std[rad], EXC_N_LIMIT/VREF 크게 둬야 고속서 프로브 안 죽음
```
설계 상세: `docs/excite_experiment_design.md`.

---

## 4. 학습

```bash
python3 mpc/train_residual.py --data ../data/hockenheim_mass.npz \
    --epochs 200 --batch-size 64 --lr 1e-3 --hidden 64 --layers 3 \
    --mode concat        # concat=조건화(8-dim). 상태-only는 무게/CoG 열 제외
```
- 시간 순서 분할(`--split-mode time`) 권장. CPU 수 초. 출력 .pt (state_dict + norm + config).
- `--jac-reg`로 Jacobian 정규화 실험 가능(→ 0으로 갈수록 FF, 발산엔 불충분).

---

## 5. 분석 스크립트

| 스크립트 | 용도 | 사용 |
|---|---|---|
| `results/identify_causal_jac.py` | 인과 조향 미분(Frisch-Waugh, model-free) | `python3 identify_causal_jac.py EXCITED.npz [CLOSEDLOOP.npz]` |
| `results/verify_decorrelation.py` | 탈공선 검증 (VIF, δ⊥, κ-잔차) | 여기 데이터에 실행 |
| `mpc_host/analyze_R_sweep.py` | R-벌점 스윕 완주·추종·조향활동 판정 | `python3 analyze_R_sweep.py logs/R_*.npy` |

**통과 기준(여기 식별):** VIF(δ)<5, δ⊥ std≥0.02, corr(δ⊥, 상태)≈0. 인과 d(Δn)/dδ가 12 m/s에서 −0.51 → 학습 −0.54와 일치.

---

## 6. 그림 재생성

- 잔차/채널/λ-스윕: 세션 스크립트(scratchpad) — figM1/T2 등.
- 미분 정확성(figT6)·seed(figT5)·권한붕괴(figT7)·발표슬라이드: venv python + Noto Sans CJK로 합성 (커밋 메시지 참조).
- 필요 폰트: `/usr/share/fonts/opentype/noto/NotoSansCJK-*.ttc` (한글).

---

## 7. 빌드 산출물 주의

- `mpc/c_generated_code/`, `mpc/*_ocp.json`, `mpc/acados_ocp_for_residual.json` = acados 코드생성 산출물(미추적). **경로가 박히므로**, 도커에서 만든 것(`/opt/mpc`)과 호스트 실행이 섞이면 ImportError. 섞였으면 삭제 후 재생성.
- 서버는 `cd $HOST/run`에서 c_generated_code를 만든다. PYTHONPATH에 `mpc/`가 있어 그쪽 오염 산출물이 우선되면 충돌 → 정리 필요.
