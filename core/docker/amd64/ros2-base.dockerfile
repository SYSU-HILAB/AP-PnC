FROM --platform=linux/amd64 ros:humble AS base

ENV DEBIAN_FRONTEND=noninteractive

# Install core build tools, C++ dependencies, and ROS2 development packages
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

# Setup decoupled workspace layout
# /workspace/src       <- host core/ros_packages/
# /workspace/bringup   <- host core/bringup/
# /workspace/artifacts <- host .artifacts/nmpc_solver/ (libnmpc_bundle.a)
WORKDIR /workspace
RUN mkdir -p /workspace/src /workspace/bringup /workspace/artifacts

ENV AP_PNC_DIR=/workspace
ENV NMPC_BUNDLE_PATH=/workspace/artifacts/libnmpc_bundle.a

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc

COPY core/docker/amd64/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Runner stage with source code compiled into the image
FROM base AS runner

COPY core/ros_packages /workspace/src/
COPY core/bringup /workspace/bringup/
COPY .artifacts/nmpc_solver/libnmpc_bundle.a /workspace/artifacts/libnmpc_bundle.a

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
