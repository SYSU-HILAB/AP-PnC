# syntax=docker/dockerfile:1
ARG TARGETARCH
FROM ap-pnc-ros2-base:${TARGETARCH}
ARG TARGETARCH
COPY core/ros_packages/planner/ /workspace/core/ros_packages/planner/
COPY core/ros_packages/nmpc/controller/ /workspace/core/ros_packages/nmpc/controller/
COPY core/ros_packages/nmpc/solver/include/ /workspace/core/ros_packages/nmpc/solver/include/
COPY core/bringup/ /workspace/core/bringup/
COPY infra/sim_infra/aerodynamics/ /workspace/infra/sim_infra/aerodynamics/
COPY infra/sim_infra/simple_sim/ /workspace/infra/sim_infra/simple_sim/
COPY .artifacts/nmpc_solver/ /workspace/.artifacts/nmpc_solver/
RUN select-nmpc-bundle
RUN . /opt/ros/humble/setup.sh && \
    colcon --log-base "$AP_PNC_DIR/.artifacts/colcon/log" build --merge-install \
    --base-paths "$AP_PNC_DIR/infra/sim_infra/simple_sim" "$AP_PNC_DIR/core/bringup" \
    --build-base "$AP_PNC_DIR/.artifacts/colcon/build" \
    --install-base "$AP_PNC_DIR/.artifacts/colcon/install" \
    --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DSIMPLE_SIM_WITH_ROS=ON -DSIMPLE_SIM_WITH_NMPC=ON \
    && rm -rf "$AP_PNC_DIR/.artifacts/colcon/build"
CMD ["ros2", "launch", "bringup", "simple_sim.launch.py", "rviz:=false"]
