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
        libceres-dev direnv

WORKDIR /ap-pnc

COPY .envrc /ap-pnc/.envrc
RUN echo 'eval "$(direnv hook bash)"' >> /root/.bashrc && \
    bash -c "direnv allow /ap-pnc"


WORKDIR /ap-pnc
COPY installations/install-acados.sh /ap-pnc/installations/install-acados.sh
COPY ./mpc_framework/ /ap-pnc/mpc_framework/
RUN direnv exec /ap-pnc ./installations/install-acados.sh

COPY . /ap-pnc
