FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]

RUN --mount=type=cache,target=/var/lib/apt/lists,id=apt_lists_cache \
    --mount=type=cache,target=/var/cache/apt/archives,id=apt_pkg_cache \
    apt update && \
    apt install -y \
    vim \
    python3-pip python3-venv \
    curl lsb-release gnupg wget \
    ros-dev-tools ccache fzf cloc \
    ros-humble-vrpn \
    ros-humble-plotjuggler* \
    libceres-dev direnv \
    ccache fzf cloc

WORKDIR /ap-pnc

COPY .envrc /ap-pnc/.envrc
RUN echo 'eval "$(direnv hook bash)"' >> /root/.bashrc && \
    bash -c "direnv allow /ap-pnc"

COPY installations/custom_funcs.sh /tmp/custom_bash_functions.sh
RUN echo "" >> /root/.bashrc && \
    cat /tmp/custom_bash_functions.sh >> /root/.bashrc && \
    rm /tmp/custom_bash_functions.sh

COPY installations/install-acados.sh /ap-pnc/installations/install-acados.sh
COPY ./mpc_framework/ /ap-pnc/mpc_framework/
COPY ./scripts/tools/ /ap-pnc/scripts/tools/
RUN direnv exec /ap-pnc ./installations/install-acados.sh

COPY . /ap-pnc/
RUN direnv exec /ap-pnc /ap-pnc/scripts/tools/cocp

COPY installations/colcon_config.yaml /.colcon/defaults.yaml
RUN source /opt/ros/humble/setup.bash && \
    direnv exec /ap-pnc colcon build --symlink-install \
    --packages-ignore aerodynamics \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    ;
