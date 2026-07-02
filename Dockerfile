# Residual MPC runtime container for TruckMaker (talks to host CarMaker via DDS)
# Base: ROS2 Foxy (Ubuntu 20.04, python 3.8) — matches host for DDS compat
FROM ros:foxy

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ---- system deps ----
RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake build-essential wget ca-certificates \
      python3-pip python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# ---- python deps ----
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel \
 && pip3 install --no-cache-dir "numpy<2" scipy casadi cython \
 && pip3 install --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu

# ---- acados (build from source) ----
WORKDIR /opt
RUN git clone https://github.com/acados/acados.git \
 && cd acados && git submodule update --recursive --init
RUN cd /opt/acados && mkdir -p build && cd build \
 && cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_HPIPM=ON .. \
 && make install -j"$(nproc)"
ENV ACADOS_SOURCE_DIR=/opt/acados
ENV LD_LIBRARY_PATH=/opt/acados/lib:${LD_LIBRARY_PATH}
# ros:foxy ships old apt python backports (importlib_metadata 1.x, numpy 1.17)
# that break setup.py egg_info and scipy/torch at runtime -> install fresh ones.
RUN pip3 install --no-cache-dir -U --ignore-installed \
      "setuptools==65.5.1" "setuptools_scm<8" wheel \
      "importlib-metadata>=4.13,<7" "numpy<2" scipy
ENV SETUPTOOLS_SCM_PRETEND_VERSION=0.0.0
RUN pip3 install --no-cache-dir --no-build-isolation -e /opt/acados/interfaces/acados_template
# the legacy editable install doesn't register importably -> put it on PYTHONPATH
ENV PYTHONPATH=/opt/acados/interfaces/acados_template:${PYTHONPATH}
# fetch tera template renderer. Ubuntu 20.04 has glibc 2.31, so the default
# (newer) tera binary fails — pin to v0.0.34 which links against old glibc.
RUN python3 -c "from acados_template.utils import get_tera; get_tera(tera_version='0.0.34', force_download=True)"

# ---- l4acados (residual learning MPC) ----
RUN cd /opt && git clone https://github.com/IntelligentControlSystems/l4acados.git \
 && cd l4acados && pip3 install --no-cache-dir -e ".[pytorch]"

# ---- ROS2 custom messages (must match host hmg_msgs) ----
COPY ros2_ws /root/mpc_ws
RUN source /opt/ros/foxy/setup.bash && cd /root/mpc_ws && colcon build

# ---- MPC source + trained model ----
COPY mpc /opt/mpc

# ---- DDS env to reach host CarMaker (run with --network host) ----
ENV ROS_DOMAIN_ID=25
ENV ROS_LOCALHOST_ONLY=0
ENV PYTHONPATH=/opt/mpc:${PYTHONPATH}

WORKDIR /opt/mpc
# auto-source ROS + workspace on interactive shells
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc \
 && echo "source /root/mpc_ws/install/setup.bash" >> /root/.bashrc
