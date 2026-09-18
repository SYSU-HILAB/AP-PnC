# nav_infra: the single navigation & interface gateway container.
#
# Base services, not research code — provides the generic flight interface
# consumed by the core container (planner + c_nmpc + px4ctrl) over the
# shared host-network ROS 2 graph:
#   - mavros2 (built from source; jammy repo ships mavros-msgs only)
#   - nmpc_bridge: /mpc_controls -> RLS throttle model + INDI thrust
#     -> AttitudeTarget; odom adapter -> /cur_state, /nmpc_bridge/odom_world
#   - flight_mux: single /mavros/setpoint_raw/attitude publisher
#
# One image for both profiles (ros2 launch bringup nav_infra.launch.py
# profile:=sim|real):
#   sim  : mavros only -> /mavros/local_position/odom (PX4 EKF2)
#   real : + livox_ros_driver2 + fast_lio + ekf_quat -> /ekf_quat/ekf_odom
FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive

# Runtime + build dependencies (PCL only lives in this image).
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
    ros-humble-ament-cmake-gtest \
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

# MAVROS GeographicLib datasets (script from the source install; downloads
# via the proxy when provided). The final check makes a silent skip fatal.
RUN export http_proxy="${HTTP_PROXY:-}" https_proxy="${HTTPS_PROXY:-}" \
    && (/opt/mavros/mavros/lib/mavros/install_geographiclib_datasets.sh \
        || geographiclib-get-geoids egm96-5) \
    && test -f /usr/share/GeographicLib/geoids/egm96-5.pgm

# Build the nav_infra packages + the shared interface contract + bringup
# (config/launch carrier). Plain install (no symlink-install) so the image
# is self-contained; the build tree is dropped afterwards.
WORKDIR /nav_ws
COPY infra/nav_infra/ros_packages/livox_ros_driver2 src/livox_ros_driver2
COPY infra/nav_infra/ros_packages/fast_lio src/fast_lio
COPY infra/nav_infra/ros_packages/ekf_quat_pose src/ekf_quat_pose
COPY infra/nav_infra/ros_packages/px4_interface src/px4_interface
# The shared interface contract is defined by the core; nav_infra consumes it.
COPY core/ros_packages/utils/msgs src/interface
COPY infra/nav_infra/bringup src/bringup

RUN bash -c ". /opt/ros/humble/setup.sh \
    && . /opt/mavros/setup.bash \
    && colcon build \
        --packages-select interface livox_ros_driver2 fast_lio ekf_quat_pose px4_interface bringup \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /opt/mavros/setup.bash" >> /root/.bashrc \
    && echo "source /nav_ws/install/setup.bash" >> /root/.bashrc

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Nodes read configs from the cwd-relative bringup/ path (repo convention).
# Default to the installed share; deployments can mount the real bringup/
# over /workspace/bringup (compose does; k3s uses a ConfigMap overlay).
RUN mkdir -p /workspace \
    && ln -s /nav_ws/install/bringup/share/bringup /workspace/bringup
WORKDIR /workspace

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /opt/mavros/setup.bash && source /nav_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "bringup", "nav_infra.launch.py", "profile:=sim"]
