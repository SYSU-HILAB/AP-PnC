FROM osrf/ros:humble-desktop

WORKDIR /workspace

COPY ros_packages/utils/ /workspace/src/utils/
COPY simulations/aerodynamics/ /workspace/src/aerodynamics/
COPY simulations/simple_sim/src/simple_sim/ /workspace/src/simple_sim/

RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-up-to simple_sim --cmake-args -DBUILD_GAZEBO_PLUGINS=OFF"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /workspace/install/setup.bash" >> /root/.bashrc

WORKDIR /workspace
