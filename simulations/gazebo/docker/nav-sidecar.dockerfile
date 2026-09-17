# nav-sidecar: the single navigation gateway container.
#
# One image for both profiles; sim vs real is a bringup selection
# (ros2 launch bringup nav_sidecar.launch.py profile:=sim|real):
#   sim  : mavros2 only -> /mavros/local_position/odom
#   real : livox + fast_lio + ekf_quat + mavros2 -> /ekf_quat/ekf_odom
#
# The ap-pnc control stack container only links mavros_msgs headers and
# consumes /mavros/* topics over the shared host-network ROS 2 graph.
FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive

# Runtime + build dependencies (PCL only lives in this image).
# NOTE: the jammy repo ships only ros-humble-mavros-msgs — the mavros
# node binaries are missing, so mavros is built from source below.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ros-dev-tools \
    libeigen3-dev \
    libyaml-cpp-dev \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    libpcl-dev \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-rosidl-default-generators \
    ros-humble-mavros-msgs \
    geographiclib-tools \
    libgeographic-dev \
    && rm -rf /var/lib/apt/lists/*

# Build mavros from source (tag aligned with the repo's mavros-msgs 2.15.1)
# and install it under /opt/mavros. Lowercase proxy vars are exported for
# python tooling (rosdep/urllib) when proxy build args are provided.
WORKDIR /tmp/mavros_ws
RUN export http_proxy="${HTTP_PROXY:-}" https_proxy="${HTTPS_PROXY:-}" no_proxy="${NO_PROXY:-}" \
    && git -c http.version=HTTP/1.1 clone --depth 1 --branch 2.15.1 \
        https://github.com/mavlink/mavros.git src/mavros \
    && for i in 1 2 3; do rosdep update && break || sleep 5; done \
    && apt-get update -qq \
    && rosdep install -y --from-paths src --ignore-src \
        --skip-keys "mavros_msgs" \
    && rm -rf /var/lib/apt/lists/* \
    && . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && cp -r install /opt/mavros \
    && cd / && rm -rf /tmp/mavros_ws

# MAVROS GeographicLib datasets (script from the source install; idempotent).
RUN /opt/mavros/lib/mavros/install_geographiclib_datasets.sh || true

# Build only the navigation packages + bringup (config/launch carrier).
WORKDIR /nav_ws
COPY ros_packages/livox_ros_driver2 src/livox_ros_driver2
COPY ros_packages/fast_lio src/fast_lio
COPY ros_packages/ekf_quat_pose src/ekf_quat_pose
COPY bringup src/bringup

RUN bash -c ". /opt/ros/humble/setup.sh \
    && . /opt/mavros/setup.bash \
    && colcon build \
        --packages-select livox_ros_driver2 fast_lio ekf_quat_pose bringup \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /opt/mavros/setup.bash" >> /root/.bashrc \
    && echo "source /nav_ws/install/setup.bash" >> /root/.bashrc

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

WORKDIR /
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /opt/mavros/setup.bash && source /nav_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "bringup", "nav_sidecar.launch.py", "profile:=sim"]
