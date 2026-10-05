# syntax=docker/dockerfile:1
# One source for both linux/arm64 and linux/amd64. The caller selects --platform.
FROM ros:humble AS base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ros-dev-tools \
    libceres-dev \
    libyaml-cpp-dev \
    libeigen3-dev \
    libspdlog-dev \
    libsqlite3-dev \
    libfmt-dev \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-std-msgs \
    ros-humble-rclcpp \
    ros-humble-rclcpp-components \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-srvs \
    ros-humble-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

ENV AP_PNC_DIR=/workspace \
    COLCON_LOG_PATH=/workspace/.artifacts/colcon/log \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp
WORKDIR /workspace
RUN mkdir -p /workspace/src /workspace/bringup /workspace/.artifacts/nmpc_solver
COPY core/docker/entrypoint.sh /entrypoint.sh
COPY core/docker/select-nmpc-bundle.sh /usr/local/bin/select-nmpc-bundle
RUN chmod +x /entrypoint.sh /usr/local/bin/select-nmpc-bundle \
    && echo 'source /entrypoint.sh true' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
