#!/bin/bash
# Run the Residual MPC node inside the container, talking to the host's
# TruckMaker cm_node over DDS. --network host + --ipc=host so Fast-DDS can use
# both UDP and shared-memory with the host process. Matching ROS_DOMAIN_ID.
# The mpc/ folder is mounted so code edits need no rebuild.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
docker run --rm -it \
  --network host \
  --ipc=host \
  -e ROS_DOMAIN_ID=25 \
  -e ROS_LOCALHOST_ONLY=0 \
  -v "${HERE}/mpc":/opt/mpc \
  mpc-tm:foxy \
  bash -lc "source /opt/ros/foxy/setup.bash \
            && source /root/mpc_ws/install/setup.bash \
            && cd /opt/mpc && python3 -u mpc_ros_node.py"
