FROM ap-pnc-ros2-base:amd64

WORKDIR /workspace

COPY ros_packages/ /workspace/src/
COPY bringup/ /workspace/bringup/
COPY simulations/aerodynamics/ /workspace/simulations/aerodynamics/
COPY .artifacts/nmpc_wrapper/libnmpc_bundle.a /workspace/artifacts/

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install \
    --base-paths /workspace/src /workspace/simulations/aerodynamics /workspace/bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
