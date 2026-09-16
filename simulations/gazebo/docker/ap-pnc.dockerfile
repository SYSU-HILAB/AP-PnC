FROM ap-pnc-ros2-base:amd64

WORKDIR /workspace

COPY ros_packages/ /workspace/src/
COPY bringup/ /workspace/bringup/
COPY .artifacts/nmpc_wrapper/libnmpc_bundle.a /workspace/artifacts/

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
