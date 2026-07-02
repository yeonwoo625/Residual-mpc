#!/bin/bash
# Train the TRUCK residual model on the collected dataset.
# Usage: train.sh [dataset.npz]   (default: ~/CarMaker/mpc_host/truck_dataset.npz)
HOST=/home/vilab/CarMaker/mpc_host
MPC=/home/vilab/CarMaker/mpc_docker/mpc
source "$HOST/venv/bin/activate"
export PYTHONPATH="$MPC:$PYTHONPATH"
cd "$MPC"
python3 train_residual.py --data "${1:-$HOST/truck_dataset.npz}" --epochs "${EPOCHS:-200}"
echo "-> 모델: $MPC/../models/residual_model.pt"
