FROM ap-pnc-ros2-base:amd64

WORKDIR /workspace

COPY core/ros_packages/ /workspace/src/
COPY core/bringup/ /workspace/bringup/
COPY infra/sim_infra/aerodynamics/ /workspace/sim_infra/aerodynamics/
COPY .artifacts/nmpc_wrapper/libnmpc_bundle.a /workspace/artifacts/

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install \
    --base-paths /workspace/src /workspace/sim_infra/aerodynamics /workspace/bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
