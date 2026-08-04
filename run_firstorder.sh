#!/bin/bash
# first-order / FF residual 구동 노드 실행 (R_sweep 실험용).
# 호스트 셸에서 export 한 RESIDUAL_*, R_DELTA, RESIDUAL_DBG_LOG 를 컨테이너로 전달한다.
# mpc_host 를 같은 경로로 마운트해서 RESIDUAL_DBG_LOG 호스트 경로가 그대로 동작.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
echo "[run] JAC_SCALE=${RESIDUAL_JAC_SCALE:-1} FF=${RESIDUAL_FF:-0} R_DELTA=${R_DELTA:-2e-4} LOG=${RESIDUAL_DBG_LOG:-<none>}"
docker run --rm -it \
  --network host \
  --ipc=host \
  -e ROS_DOMAIN_ID=25 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/opt/mpc/fastdds_udp.xml \
  -e RESIDUAL_MODEL_PATH="${RESIDUAL_MODEL_PATH:-/opt/mpc/residual_model.pt}" \
  -e RESIDUAL_JAC_SCALE="${RESIDUAL_JAC_SCALE:-1}" \
  -e RESIDUAL_FF="${RESIDUAL_FF:-0}" \
  -e RESIDUAL_MASK_JAC="${RESIDUAL_MASK_JAC:-}" \
  -e RESIDUAL_DBG_LOG="${RESIDUAL_DBG_LOG:-}" \
  -e REFERENCE_PATH="${REFERENCE_PATH:-/home/vilab/CarMaker/mpc_host/hockenheim_waypoints_1lap.npy}" \
  -e TARGET_SPEED="${TARGET_SPEED:-12}" \
  -e MASS="${MASS:-48000}" \
  -e COG_X="${COG_X:-4.330}" \
  -e R_DELTA="${R_DELTA:-2e-4}" \
  -e R_D="${R_D:-2e-4}" \
  -v "${HERE}/mpc":/opt/mpc \
  -v /home/vilab/CarMaker/mpc_host:/home/vilab/CarMaker/mpc_host \
  mpc-tm:foxy \
  bash -lc "source /opt/ros/foxy/setup.bash \
            && source /root/mpc_ws/install/setup.bash \
            && cd /opt/mpc && python3 -u mpc_ros_node.py"
