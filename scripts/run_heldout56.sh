#!/bin/bash
# held-out 56t 폐루프 주행 러너 — env 실수를 막으려고 한 줄로 묶었다.
#
#   ./scripts/run_heldout56.sh state 0     # 상태-only(6dim), 시드 0
#   ./scripts/run_heldout56.sh cond  2     # 조건화(8dim),   시드 2
#
# 모델은 32/40/48t 로만 학습됨 (56t 는 학습·val·정규화 어디에도 없음).
# 주행 후 서버 터미널에서 Ctrl-C -> LOG_TRAJ 저장.
set -e
ARM="$1"; SEED="$2"
if [[ "$ARM" != "state" && "$ARM" != "cond" ]] || [[ -z "$SEED" ]]; then
    echo "사용법: $0 {state|cond} {0|1|2}"; exit 1
fi

ROOT=/home/vilab/CarMaker/mpc_docker
HOST=/home/vilab/CarMaker/mpc_host
MODEL="$ROOT/models/residual_heldout56_${ARM}_s${SEED}.pt"
[[ -f "$MODEL" ]] || { echo "모델 없음: $MODEL"; exit 1; }

export USE_RESIDUAL=1 RESIDUAL_FF=1 RESIDUAL_SCALE=0.3
export REFERENCE_PATH="$HOST/hockenheim_waypoints_1lap.npy"
export TARGET_SPEED=10                      # 기존 56t 데이터와 같은 속도
export RESIDUAL_MODEL_PATH="$MODEL"
export LOG_TRAJ="$HOST/heldout56_${ARM}_s${SEED}.npy"

if [[ "$ARM" == "cond" ]]; then
    export MASS=56000 COG_X=4.330           # 8dim 모델은 컨텍스트 필수
else
    unset MASS COG_X                        # 6dim 모델은 반드시 해제
fi

echo "=============================================="
echo " arm=$ARM  seed=$SEED  speed=$TARGET_SPEED"
echo " model : $(basename "$MODEL")"
echo " log   : $LOG_TRAJ"
echo " MASS  : ${MASS:-(미설정)}"
echo "=============================================="
exec "$ROOT/scripts/1_server.sh"
