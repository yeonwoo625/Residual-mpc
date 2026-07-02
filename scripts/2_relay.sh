#!/bin/bash
# [2] MPC ROS 릴레이 — /hmg_vehicle_state <-> 솔버 <-> /hmg_ctrl 중계.
#     서버 빌드 완료 + TruckMaker Start(데이터 흐름) 후 다른 터미널에서 실행.
MPC=/home/vilab/CarMaker/mpc_docker/mpc
source /opt/ros/foxy/setup.bash
source /home/vilab/CarMaker/TMROS2/ros/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=25 MPC_SOCK=/tmp/mpc_sock
exec python3 -u "$MPC/mpc_ros_relay.py"
