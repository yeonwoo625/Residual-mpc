#!/bin/bash
# [1] MPC 솔버 서버 — ref_waypoints.npy로 MPC 빌드(~40s) 후 소켓 대기.
#     제일 먼저, 자기 터미널에서 실행. (acados, rclpy 없음)
HOST=/home/vilab/CarMaker/mpc_host
MPC=/home/vilab/CarMaker/mpc_docker/mpc
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
source "$HOST/venv/bin/activate"
export ACADOS_SOURCE_DIR="$HOST/acados"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$MPC:$ACADOS_SOURCE_DIR/interfaces/acados_template"
export MPC_SOCK=/tmp/mpc_sock
export REFERENCE_PATH="${REFERENCE_PATH:-$HOST/ref_waypoints.npy}"
export TARGET_SPEED="${TARGET_SPEED:-4.0}"
export STEER_RATIO="${STEER_RATIO:-10.0}"
mkdir -p "$HOST/run"; cd "$HOST/run"
exec python3 -u "$MPC/mpc_solver_server.py"
