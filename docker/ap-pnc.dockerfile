FROM osrf/ros:humble-desktop

RUN apt update && apt install -y \
    vim \
    python3-pip python3-venv \
    curl lsb-release gnupg wget

WORKDIR /ap-pnc

RUN apt install -y libceres-dev direnv
COPY installations /ap-pnc/installations
COPY .envrc /ap-pnc/.envrc
RUN echo 'eval "$(direnv hook bash)"' >> /root/.bashrc && \
    bash -c "source /root/.bashrc && direnv allow /ap-pnc"


RUN apt install -y ros-dev-tools ccache fzf cloc ros-humble-vrpn ros-humble-plotju*

WORKDIR /ap-pnc
RUN bash installations/install-acados.sh
# COPY . /ap-pnc