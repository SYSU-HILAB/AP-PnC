# syntax=docker/dockerfile:1
ARG TARGETARCH
FROM simple-sim:${TARGETARCH}
# The interactive image does not need the offline exporter sources.
RUN rm -rf "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack"
RUN if ! dpkg -s python3-numpy python3-yaml ros-humble-tf2-ros >/dev/null 2>&1; then \
    apt-get update && apt-get install -y --no-install-recommends \
      python3-numpy python3-yaml ros-humble-tf2-ros && rm -rf /var/lib/apt/lists/*; fi
COPY infra/sim_infra/lidar_sim/ /workspace/infra/sim_infra/lidar_sim/
COPY core/bringup/ /workspace/core/bringup/
RUN . /opt/ros/humble/setup.sh && \
    colcon --log-base "$AP_PNC_DIR/.artifacts/dev-sim/log" build --merge-install \
    --base-paths "$AP_PNC_DIR/infra/sim_infra/lidar_sim" "$AP_PNC_DIR/core/bringup" \
    --build-base "$AP_PNC_DIR/.artifacts/dev-sim/build" \
    --install-base "$AP_PNC_DIR/.artifacts/colcon/install" \
    --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF && \
    rm -rf "$AP_PNC_DIR/.artifacts/dev-sim/build"
CMD ["ros2", "launch", "bringup", "dev_sim.launch.py"]
