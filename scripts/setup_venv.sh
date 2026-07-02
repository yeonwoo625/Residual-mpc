#!/bin/bash
# Build an isolated host venv for the Residual MPC (reuses acados/l4acados
# extracted from the container). The host lacks python3-venv's bundled pip, so
# create the venv without pip and bootstrap pip via get-pip (no apt/sudo).
# rclpy + hmg_msgs come from the host ROS at runtime via PYTHONPATH.
set -e
HOST=/home/vilab/CarMaker/mpc_host

rm -rf "$HOST/venv"
python3 -m venv --without-pip "$HOST/venv"
source "$HOST/venv/bin/activate"

# bootstrap pip (python 3.8 pinned get-pip)
curl -sS https://bootstrap.pypa.io/pip/3.8/get-pip.py -o /tmp/get-pip.py
python /tmp/get-pip.py

pip install "setuptools==65.5.1" "setuptools_scm<8" wheel "importlib-metadata>=4.13,<7"
pip install "numpy<2" scipy casadi cython
pip install torch --index-url https://download.pytorch.org/whl/cpu

export SETUPTOOLS_SCM_PRETEND_VERSION=0.0.0
pip install --no-build-isolation -e "$HOST/acados/interfaces/acados_template"
pip install -e "$HOST/l4acados[pytorch]"

echo "VENV_SETUP_DONE"
