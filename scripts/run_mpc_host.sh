#!/bin/bash
# Run the Residual MPC node on the HOST (isolated venv) so it talks to cm_node
# over host-to-host DDS (no container boundary). rclpy + hmg_msgs from host ROS.
set -e
HOST=/home/vilab/CarMaker/mpc_host
MPC=/home/vilab/CarMaker/mpc_docker/mpc

# single-thread all BLAS backends: torch/numpy(OpenBLAS) + acados(blasfeo)
# sharing a process otherwise segfaults after a while.
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1 NUMEXPR_NUM_THREADS=1 BLIS_NUM_THREADS=1

source /opt/ros/foxy/setup.bash
source /home/vilab/CarMaker/TMROS2/ros/ros2_ws/install/setup.bash   # hmg_msgs
source "$HOST/venv/bin/activate"

export ACADOS_SOURCE_DIR="$HOST/acados"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$MPC:$ACADOS_SOURCE_DIR/interfaces/acados_template:$PYTHONPATH"
export ROS_DOMAIN_ID=25

mkdir -p "$HOST/run"
cd "$HOST/run"
exec python3 -u "$MPC/mpc_ros_node.py"
