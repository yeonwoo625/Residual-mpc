#!/bin/bash
# [1b] DATA COLLECTOR — drives the truck with the NOMINAL MPC and records
# (obs, residual) for residual learning. Use INSTEAD of 1_server.sh:
#   collect.sh   ->  Start TruckMaker  ->  2_relay.sh   (then drive)
# Ctrl-C the collector to save. Re-run on different roads/speeds to add coverage
# (data is appended to OUT_NPZ).
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
export OUT_NPZ="${OUT_NPZ:-$HOST/truck_dataset.npz}"
mkdir -p "$HOST/run"; cd "$HOST/run"
exec python3 -u "$MPC/mpc_data_collector.py"
