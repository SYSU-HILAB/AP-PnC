FROM osrf/ros:humble-desktop

RUN apt update && apt install -y \
    vim \
    python3-pip python3-venv \
    curl lsb-release gnupg wget

WORKDIR /ap-pnc

COPY px4_vtol/Tools /ap-pnc/px4_vtol/Tools
RUN bash /ap-pnc/px4_vtol/Tools/setup/ubuntu.sh

COPY px4_vtol /ap-pnc/px4_vtol
RUN bash /ap-pnc/px4_vtol/install-dds-agent.bash
