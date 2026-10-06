# syntax=docker/dockerfile:1
# Both stages support native ARM64/AMD64. sim excludes hardware/LIO dependencies.
FROM ros:humble-ros-base-jammy AS mavros-base
ARG DEBIAN_FRONTEND=noninteractive
ARG MAVROS_VERSION=2.15.1
ENV AP_PNC_DIR=/workspace COLCON_LOG_PATH=/workspace/.artifacts/colcon/log \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp
WORKDIR /workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ros-dev-tools libeigen3-dev libyaml-cpp-dev \
    ros-humble-rmw-fastrtps-cpp ros-humble-tf2 ros-humble-tf2-ros \
    ros-humble-rosidl-default-generators ros-humble-ament-cmake-gtest \
    "ros-humble-mavros-msgs=${MAVROS_VERSION}-*" geographiclib-tools libgeographic-dev \
    && rm -rf /var/lib/apt/lists/*
RUN if [ -n "${HTTP_PROXY:-}" ]; then export http_proxy="$HTTP_PROXY"; fi \
    && if [ -n "${HTTPS_PROXY:-}" ]; then export https_proxy="$HTTPS_PROXY"; fi \
    && if [ -n "${NO_PROXY:-}" ]; then export no_proxy="$NO_PROXY"; fi \
    && git -c http.version=HTTP/1.1 clone --depth 1 --branch "$MAVROS_VERSION" \
        https://github.com/mavlink/mavros.git "$AP_PNC_DIR/.artifacts/mavros_src/src/mavros" \
    && for attempt in 1 2 3; do \
        if rosdep update --rosdistro humble; then break; fi; \
        [ "$attempt" != 3 ] || exit 1; sleep 5; \
       done \
    && apt-get update -qq \
    && rosdep install -y --from-paths "$AP_PNC_DIR/.artifacts/mavros_src/src" --ignore-src --skip-keys mavros_msgs \
    && rm -rf /var/lib/apt/lists/* \
    && . /opt/ros/humble/setup.sh \
    && colcon --log-base "$AP_PNC_DIR/.artifacts/colcon/mavros/log" build \
        --base-paths "$AP_PNC_DIR/.artifacts/mavros_src/src" \
        --build-base "$AP_PNC_DIR/.artifacts/colcon/mavros/build" \
        --install-base "$AP_PNC_DIR/.artifacts/colcon/mavros/install" \
        --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf "$AP_PNC_DIR/.artifacts/colcon/mavros/build"
RUN if [ -n "${HTTP_PROXY:-}" ]; then export http_proxy="$HTTP_PROXY"; fi \
    && if [ -n "${HTTPS_PROXY:-}" ]; then export https_proxy="$HTTPS_PROXY"; fi \
    && ("$AP_PNC_DIR/.artifacts/colcon/mavros/install/mavros/lib/mavros/install_geographiclib_datasets.sh" \
        || geographiclib-get-geoids egm96-5) \
    && test -f /usr/share/GeographicLib/geoids/egm96-5.pgm
COPY core/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

FROM mavros-base AS sim
COPY infra/nav_infra/ros_packages/px4ctrl /workspace/src/px4ctrl
COPY infra/nav_infra/ros_packages/quadrotor_msgs /workspace/src/quadrotor_msgs
COPY core/ros_packages/interface /workspace/src/interface
COPY infra/nav_infra/bringup /workspace/bringup
RUN /entrypoint.sh bash -c 'colcon --log-base "$AP_PNC_DIR/.artifacts/colcon/log" build \
    --base-paths "$AP_PNC_DIR/src" "$AP_PNC_DIR/bringup" \
    --build-base "$AP_PNC_DIR/.artifacts/colcon/build" \
    --install-base "$AP_PNC_DIR/.artifacts/colcon/install" \
    --packages-select interface quadrotor_msgs px4ctrl bringup \
    --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf "$AP_PNC_DIR/.artifacts/colcon/build"'
COPY infra/sim_infra/gazebo/scripts/verify_runtime.py /workspace/infra/sim_infra/gazebo/scripts/verify_runtime.py
CMD ["ros2", "launch", "bringup", "nav_infra.launch.py", "profile:=sim"]

FROM sim AS real
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-pcl-ros ros-humble-pcl-conversions libpcl-dev \
    && rm -rf /var/lib/apt/lists/*
COPY infra/nav_infra/ros_packages/livox_ros_driver2 /workspace/src/livox_ros_driver2
COPY infra/nav_infra/ros_packages/fast_lio /workspace/src/fast_lio
COPY infra/nav_infra/ros_packages/ekf_quat_pose /workspace/src/ekf_quat_pose
RUN /entrypoint.sh bash -c 'colcon --log-base "$AP_PNC_DIR/.artifacts/colcon/log" build \
    --base-paths "$AP_PNC_DIR/src" "$AP_PNC_DIR/bringup" \
    --build-base "$AP_PNC_DIR/.artifacts/colcon/build" \
    --install-base "$AP_PNC_DIR/.artifacts/colcon/install" \
    --packages-select livox_ros_driver2 fast_lio ekf_quat_pose \
    --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf "$AP_PNC_DIR/.artifacts/colcon/build"'
CMD ["ros2", "launch", "bringup", "nav_infra.launch.py", "profile:=real"]
