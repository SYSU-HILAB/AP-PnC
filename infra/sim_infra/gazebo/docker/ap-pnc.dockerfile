FROM ap-pnc-ros2-base:amd64

WORKDIR /workspace

COPY core/ros_packages/ /workspace/src/
COPY core/bringup/ /workspace/bringup/
COPY infra/sim_infra/aerodynamics/ /workspace/infra/sim_infra/aerodynamics/
COPY .artifacts/nmpc_solver/libnmpc_bundle.a /workspace/.artifacts/nmpc_solver/

RUN . /opt/ros/humble/setup.sh && \
    colcon --log-base ${AP_PNC_DIR}/.artifacts/colcon/log build --symlink-install \
    --base-paths ${AP_PNC_DIR}/src ${AP_PNC_DIR}/infra/sim_infra/aerodynamics ${AP_PNC_DIR}/bringup \
    --build-base ${AP_PNC_DIR}/.artifacts/colcon/build \
    --install-base ${AP_PNC_DIR}/.artifacts/colcon/install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
