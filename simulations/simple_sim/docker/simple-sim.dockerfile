FROM osrf/ros:humble-desktop

# Eigen3 ships with ros-humble-desktop; no extra apt deps needed.

WORKDIR /workspace

COPY simulations/simple_sim/src/simple_sim/ /workspace/src/simple_sim/

RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-select simple_sim"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /workspace/install/setup.bash" >> /root/.bashrc

WORKDIR /workspace
