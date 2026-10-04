FROM ros:humble-ros-base

ENV AP_PNC_DIR=/workspace
ENV COLCON_LOG_PATH=/workspace/.artifacts/colcon/log
WORKDIR /workspace

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions \
    libeigen3-dev libyaml-cpp-dev ros-humble-rviz2 \
    ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-std-srvs ros-humble-rosgraph-msgs \
    && rm -rf /var/lib/apt/lists/*

# Keep the same owner/source layout as the repository.
COPY core/ros_packages/planner/ /workspace/core/ros_packages/planner/
COPY core/ros_packages/nmpc/controller/ /workspace/core/ros_packages/nmpc/controller/
COPY core/ros_packages/nmpc/solver/include/ /workspace/core/ros_packages/nmpc/solver/include/
COPY core/bringup/ /workspace/core/bringup/
COPY infra/sim_infra/aerodynamics/ /workspace/infra/sim_infra/aerodynamics/
COPY infra/sim_infra/simple_sim/ /workspace/infra/sim_infra/simple_sim/
# Regenerate on Linux after solver API changes; this is a build prerequisite.
COPY .artifacts/nmpc_solver/libnmpc_bundle.a /workspace/.artifacts/nmpc_solver/libnmpc_bundle.a

RUN bash -c 'source /opt/ros/humble/setup.bash && \
    colcon build --merge-install \
    --base-paths /workspace/infra/sim_infra/simple_sim /workspace/core/bringup \
    --build-base /workspace/.artifacts/colcon/build \
    --install-base /workspace/.artifacts/colcon/install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
                 -DSIMPLE_SIM_WITH_ROS=ON -DSIMPLE_SIM_WITH_NMPC=ON'

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /workspace/.artifacts/colcon/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
