# syntax=docker/dockerfile:1
# The CLI builds the matching shared base first. TARGETARCH is provided by BuildKit.
ARG TARGETARCH
FROM ap-pnc-ros2-base:${TARGETARCH}
ARG TARGETARCH

COPY core/ros_packages/ /workspace/src/
COPY core/bringup/ /workspace/bringup/
COPY infra/sim_infra/aerodynamics/ /workspace/infra/sim_infra/aerodynamics/
COPY .artifacts/nmpc_solver/ /workspace/.artifacts/nmpc_solver/
RUN select-nmpc-bundle
RUN . /opt/ros/humble/setup.sh && \
    colcon --log-base "$AP_PNC_DIR/.artifacts/colcon/log" build \
    --base-paths "$AP_PNC_DIR/src" "$AP_PNC_DIR/bringup" "$AP_PNC_DIR/infra/sim_infra/aerodynamics" \
    --build-base "$AP_PNC_DIR/.artifacts/colcon/build" \
    --install-base "$AP_PNC_DIR/.artifacts/colcon/install" \
    --packages-select interface nmpc planner bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf "$AP_PNC_DIR/.artifacts/colcon/build"
CMD ["ros2", "launch", "bringup", "bringup.launch.py"]
