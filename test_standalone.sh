#!/bin/bash
# Validate the MPC stack (acados + l4acados + torch + residual model) WITHOUT
# CarMaker: runs the built-in straight-line (kappa=0) solve test.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
docker run --rm -it \
  -v "${HERE}/mpc":/opt/mpc \
  mpc-tm:foxy \
  bash -lc "cd /opt/mpc && python3 mpc_solver_residual.py"
