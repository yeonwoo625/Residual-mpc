#!/bin/bash
# 적재별 ablation 주행 러너 (전 적재 성능표 / 조건화 ablation 용).
#
#   ./scripts/run_ablation.sh nomass 48 1     # 무게-blind(6dim), 48t, 시드1
#   ./scripts/run_ablation.sh cond   48 1     # 조건화(8dim),    48t, 시드1
#
# ⚠️ TruckMaker Loads 를 해당 적재로 먼저 바꿀 것 (payload = 총질량 - 32000):
#      32t -> 0(비움)   40t -> 8000   48t -> 16000   56t -> 24000   (x=4.33)
# 주행 후 서버 터미널 Ctrl-C -> LOG_TRAJ 저장.
set -e
ARM="$1"; MASS_T="$2"; SEED="$3"
if [[ "$ARM" != "nomass" && "$ARM" != "cond" && "$ARM" != "shufmass" ]] \
   || [[ -z "$MASS_T" || -z "$SEED" ]]; then
    echo "사용법: $0 {nomass|cond|shufmass} {32|40|48|56} {0|1|2}"
    echo "  shufmass = 무게 열을 섞어 학습한 플라시보 (8dim, 정보량 0)"; exit 1
fi

ROOT=/home/vilab/CarMaker/mpc_docker
HOST=/home/vilab/CarMaker/mpc_host
MODEL="$ROOT/mpc/residual_model_${ARM}_s${SEED}.pt"
[[ -f "$MODEL" ]] || { echo "모델 없음: $MODEL"; exit 1; }

export USE_RESIDUAL=1 RESIDUAL_FF=1 RESIDUAL_SCALE=0.3
export REFERENCE_PATH="$HOST/hockenheim_waypoints_1lap.npy"
export TARGET_SPEED=10                      # 기존 ablation/traj 데이터와 동일
export RESIDUAL_MODEL_PATH="$MODEL"
export LOG_TRAJ="$HOST/abl_${ARM}_${MASS_T}_s${SEED}.npy"

if [[ "$ARM" == "cond" || "$ARM" == "shufmass" ]]; then
    export MASS=$((MASS_T * 1000)) COG_X=4.330    # 8dim 모델 (플라시보도 실제 무게를 준다)
else
    unset MASS COG_X
fi

PAYLOAD=$(( MASS_T * 1000 - 32000 ))
echo "=============================================="
echo " arm=$ARM  적재=${MASS_T}t  seed=$SEED  speed=$TARGET_SPEED"
echo " model : $(basename "$MODEL")"
echo " log   : $LOG_TRAJ"
echo " MASS  : ${MASS:-(미설정)}"
echo " ★ TruckMaker Loads = ${PAYLOAD} kg @ x=4.33 인지 확인!"
echo "=============================================="
exec "$ROOT/scripts/1_server.sh"
