# TruckMaker Residual MPC

IPG TruckMaker(8×4 트럭)를 **Frenet 프레임 Residual MPC**로 제어하는 코드.
Nominal(운동학 bicycle) 모델 + 학습된 잔차(MLP)를 acados/l4acados로 결합해 곡선을 추종한다.

> ⚠️ **이 repo는 private로 유지하세요.** CarMaker(IPG) 라이선스 코드는 포함하지 않지만,
> 관련 브릿지(`cmnode_hellocm`)는 IPG CMRosIF 파생이라 여기 넣지 않았습니다.

---

## 구조

```
mpc/                     ← MPC 소스코드 (16개 .py)
  bicycle_model.py         Frenet 운동학 모델 (nominal, 트럭 재동정 계수)
  acados_settings.py       acados OCP 정의
  reference_utils.py       경로 스플라인 / Frenet 변환
  mpc_solver.py            MPCSolver (nominal)
  mpc_solver_residual.py   MPCSolverResidual (nominal + 잔차, l4acados)
  residual_model.py        잔차 MLP (6→64→64→4, Tanh)
  residual_model_wrapper.py  정규화 + scale + clamp 래퍼
  kappa_feature_selector.py  l4acados 입력 feature(kappa) 선택
  dynamics_predict.py      nominal 1-스텝 예측 (잔차 계산용)
  train_residual.py        잔차 MLP 학습 (weight_decay/jac_reg 지원)
  mpc_data_collector.py    데이터 수집기 (nominal 주행 + 잔차 기록 + 여진)
  sysid_longitudinal.py    종방향 force 계수 재동정
  renominalize.py          데이터셋을 새 nominal 기준으로 변환
  mpc_solver_server.py     솔버 서버 (acados, 유닉스 소켓)
  mpc_ros_relay.py         ROS↔소켓 릴레이 (rclpy)
  mpc_ros_node.py          단일프로세스 노드 (구버전, 미사용)
scripts/                 ← 실행 스크립트 (.sh) + record_path.py
ros2_ws/src/hmg_msgs/    ← 커스텀 ROS 메시지 (VehicleState, TestMsgs)
models/                  ← 학습된 잔차 모델 (.pt)
data/                    ← ref_waypoints.npy, truck_dataset_combined.npz
Dockerfile               ← 컨테이너 빌드 (현재는 호스트 venv 사용)
fastdds_*.xml            ← DDS 설정 실험본
```

## 의존성

**pip** (`requirements.txt`): numpy(<2), scipy, casadi, torch, matplotlib
**별도 설치**:
- `acados` + `acados_template` — 소스 빌드 (https://docs.acados.org)
- `l4acados` — 소스 설치 (ResidualLearningMPC)
- ROS2 **Foxy** + `rclpy` + `hmg_msgs`(colcon 빌드: `ros2_ws/src/hmg_msgs`)
- **CarMaker CMRosIF 브릿지** `cmnode_hellocm` — `/hmg_vehicle_state` 발행 + `/hmg_ctrl` 구독 (TruckMaker측, IPG 파생이라 미포함)

## 아키텍처 (실행 시)

```
[TruckMaker cm_node] --/hmg_vehicle_state--> [relay(rclpy)] --socket--> [server(acados)]
                     <----/hmg_ctrl---------              <--socket--
```
rclpy(fast-DDS)와 acados를 **한 프로세스에 두면 크래시** → 서버/릴레이 분리.

## 워크플로우

```bash
# 0) 환경: scripts/setup_venv.sh 로 venv 구성 (torch/casadi/acados_template/l4acados)
# 1) 참조경로 취득: record_path.py (IPGDriver 주행 궤적 기록)
# 2) 데이터 수집:  collect.sh   → Start(TruckMaker) → 2_relay.sh → 주행 → Ctrl-C
#    (여진: EXCITE_DELTA=0.015 EXCITE_D=0.03 붙이면 off-nominal 커버)
# 3) 종방향 재동정: sysid.sh (Cm1,Cm2,Cr2,Cr0) → bicycle_model.py/dynamics_predict.py 갱신
# 4) 데이터 재nominal화: renominalize.py
# 5) 학습:        train.sh [dataset.npz]  → models/residual_model.pt  (~20초, CPU)
# 6) 실행:        1_server.sh (+USE_RESIDUAL=1) & 2_relay.sh
```
> ⚠️ 서버/릴레이는 이 환경에서 **`setsid`로 띄워야** 세션 정리에 안 죽음.

## 주요 결과 / 교훈

- **재동정 nominal**: 완만곡선에서 **횡오차 <3cm** (안정, cm급). 종방향 재동정으로 속도편향 −38%.
- **residual (완만맵)**: nominal 대비 **유의미한 향상 없음** — 모델이 이미 정확한 영역엔 잔차가 보탤 게 없음.
- **Δv(종방향) 채널**: 기어 변속 불연속으로 잔차가 거칠어 **MPC 발산** → `B_MATRIX`를 **lateral-only(Δv off)**로 두면 안정.
- **결론**: residual은 **kinematic이 깨지는 급커브·고속(타이어 슬립)**에서 가치. 무게·CoG는 학습보다 **nominal 물리 파라미터**로 처리.

## 라이선스 주의

CarMaker/IPG(TMROS2, CMROS2, cmnode) 코드는 **라이선스 대상**이라 이 repo에 포함하지 않음.
재현하려면 IPG CarMaker 라이선스 + CMRosIF 브릿지가 별도로 필요.
