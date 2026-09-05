FROM osrf/ros:humble-desktop

RUN apt update && apt install -y \
    ros-dev-tools libceres-dev

WORKDIR /ap-pnc

COPY src/ src/
COPY config/ ./config/

ENV AP_PNC_DIR=/ap-pnc
ENV NMPC_BUNDLE_PATH=/ap-pnc/build/nmpc_wrapper/libnmpc_bundle.a

RUN mkdir -p build/nmpc_wrapper
COPY build/nmpc_wrapper/libnmpc_bundle.a build/nmpc_wrapper/

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install \
    --cmake-args -DBUILD_GAZEBO_PLUGINS=OFF -DCMAKE_BUILD_TYPE=Release
