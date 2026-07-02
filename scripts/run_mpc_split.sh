#!/bin/bash
# Split MPC: solver server (acados, venv) + ROS relay (rclpy, system python).
# Keeping fast-DDS and acados in separate processes avoids the segfault.
set -e
HOST=/home/vilab/CarMaker/mpc_host
MPC=/home/vilab/CarMaker/mpc_docker/mpc
export MPC_SOCK=/tmp/mpc_sock
export REFERENCE_PATH="${REFERENCE_PATH:-$HOST/ref_waypoints.npy}"
export TARGET_SPEED="${TARGET_SPEED:-4.0}"
export STEER_RATIO="${STEER_RATIO:-10.0}"

# ---- solver server: venv python + acados, NO rclpy ----
(
  export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
  source "$HOST/venv/bin/activate"
  export ACADOS_SOURCE_DIR="$HOST/acados"
  export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH"
  export PYTHONPATH="$MPC:$ACADOS_SOURCE_DIR/interfaces/acados_template"
  mkdir -p "$HOST/run"; cd "$HOST/run"
  exec python3 -u "$MPC/mpc_solver_server.py"
) > "$HOST/solver.log" 2>&1 &
SERVER_PID=$!
echo "[split] solver server PID=$SERVER_PID -> $HOST/solver.log (building ~40s)"
trap 'kill $SERVER_PID 2>/dev/null' EXIT

# ---- ROS relay: SYSTEM python + rclpy, NO acados/venv ----
source /opt/ros/foxy/setup.bash
source /home/vilab/CarMaker/TMROS2/ros/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=25
unset FASTRTPS_DEFAULT_PROFILES_FILE ROS_LOCALHOST_ONLY
exec python3 -u "$MPC/mpc_ros_relay.py"
