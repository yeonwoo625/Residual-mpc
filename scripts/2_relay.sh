#!/bin/bash
# [2] MPC ROS 릴레이 — /hmg_vehicle_state <-> 솔버 <-> /hmg_ctrl 중계.
#     서버 빌드 완료 + TruckMaker Start(데이터 흐름) 후 다른 터미널에서 실행.
MPC=/home/vilab/CarMaker/mpc_docker/mpc
source /opt/ros/foxy/setup.bash
source /home/vilab/CarMaker/TMROS2/ros/ros2_ws/install/setup.bash
# cm_node(브릿지)와 같은 도메인이어야 함. 공유 PC라 .bashrc 값(현재 27)이 바뀔 수
# 있으니, 셸 환경 값을 상속(기본 27). cm_node도 같은 .bashrc를 물려받아 일치.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-27} MPC_SOCK=/tmp/mpc_sock
exec python3 -u "$MPC/mpc_ros_relay.py"
