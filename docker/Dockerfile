# vector-tern dev/sim image: ROS 2 Humble + Gazebo Harmonic + PX4 SITL + Micro XRCE-DDS Agent.
# Target host: Ubuntu 22.04 x86_64 with an NVIDIA GPU (needed for cuVSLAM / nvblox).
FROM osrf/ros:humble-desktop

ARG PX4_VERSION=v1.15.4
ARG PX4_MSGS_BRANCH=release/1.15
ARG XRCE_AGENT_VERSION=v2.4.2
# Set to 1 to install Isaac ROS Visual SLAM (cuVSLAM) from NVIDIA's apt repo.
# Requires the NVIDIA container runtime and a CUDA-capable GPU at run time.
ARG INSTALL_ISAAC_ROS=0

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo curl gnupg lsb-release git wget cmake build-essential \
        python3-pip python3-venv python3-colcon-common-extensions \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# --- Gazebo Harmonic + ros_gz for Humble (OSRF repo) ---
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic ros-humble-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*

# --- PX4 SITL ---
RUN git clone --recursive --depth 1 -b ${PX4_VERSION} https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
    && cd /opt/PX4-Autopilot \
    && bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools \
    && make px4_sitl \
    && rm -rf /var/lib/apt/lists/*
ENV PX4_DIR=/opt/PX4-Autopilot

# --- Micro XRCE-DDS Agent (PX4 <-> ROS 2) ---
RUN git clone --depth 1 -b ${XRCE_AGENT_VERSION} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
    && cmake -S /tmp/agent -B /tmp/agent/build \
    && cmake --build /tmp/agent/build -j"$(nproc)" \
    && cmake --install /tmp/agent/build \
    && ldconfig && rm -rf /tmp/agent

# --- px4_msgs underlay (must match PX4_VERSION) ---
RUN mkdir -p /opt/px4_ws/src \
    && git clone --depth 1 -b ${PX4_MSGS_BRANCH} https://github.com/PX4/px4_msgs.git /opt/px4_ws/src/px4_msgs \
    && source /opt/ros/humble/setup.bash \
    && cd /opt/px4_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- Optional: Isaac ROS Visual SLAM (cuVSLAM) ---
RUN if [ "${INSTALL_ISAAC_ROS}" = "1" ]; then \
        k=/usr/share/keyrings/nvidia-isaac-ros.gpg \
        && curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor -o "$k" \
        && echo "deb [signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" \
            > /etc/apt/sources.list.d/nvidia-isaac-ros.list \
        && apt-get update && apt-get install -y --no-install-recommends ros-humble-isaac-ros-visual-slam \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# --- Python tools: evaluation + world/texture generation ---
RUN pip3 install --no-cache-dir evo numpy pillow

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /opt/px4_ws/install/setup.bash" >> /root/.bashrc \
    && echo "[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /workspace
