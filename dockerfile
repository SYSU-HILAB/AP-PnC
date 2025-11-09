FROM osrf/ros:humble-desktop

ENV http_proxy=http://127.0.0.1:8889 \
    https_proxy=http://127.0.0.1:8889 \
    HTTP_PROXY=http://127.0.0.1:8889 \
    HTTPS_PROXY=http://127.0.0.1:8889 \
    no_proxy=localhost,127.0.0.1,::1 \
    NO_PROXY=localhost,127.0.0.1,::1

RUN apt update && apt install -y \
    vim \
    python3-pip python3-venv \
    curl lsb-release gnupg wget

COPY . /ap-pnc

RUN bash ap-pnc/px4-vtol/Tools/setup/ubuntu.sh
