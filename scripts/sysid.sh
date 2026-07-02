#!/bin/bash
# 종방향 force 계수 재동정 (오프라인 — TruckMaker 불필요).
# Usage: sysid.sh [dataset.npz]   (default: truck_dataset.npz)
HOST=/home/vilab/CarMaker/mpc_host
MPC=/home/vilab/CarMaker/mpc_docker/mpc
source "$HOST/venv/bin/activate"
export PYTHONPATH="$MPC:$PYTHONPATH"
python3 "$MPC/sysid_longitudinal.py" "${1:-$HOST/truck_dataset.npz}"
