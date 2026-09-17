# PX4 SITL simulator container — stock upstream firmware (no forks).
#
# Mirrors the upstream v1.17 container recipe (ubuntu:24.04 +
# Tools/setup/ubuntu.sh) but selects Gazebo Jetty (gz-sim9) instead of the
# script default Harmonic — v1.17 adds Jetty support. PX4 itself is pinned
# to a release tag (ARG PX4_VERSION).
#
# Run the SITL inside with:  cd /home/px4/PX4-Autopilot && make px4_sitl gz_x500
# MAVLink: TCP server on 4560 (nav-sidecar mavros fcu_url) plus the default
# UDP broadcasters for QGC.
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/lib/apt/lists,id=apt_cache_pxsitl,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    git \
    wget \
    curl \
    sudo \
    lsb-release \
    gnupg \
    vim \
    file \
    xz-utils \
    locales \
    python3-pip \
    python3-venv \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Create px4 user with UID 1002; ubuntu.sh is invoked through sudo.
RUN groupadd -r px4 && useradd -m -g px4 -u 1002 -s /bin/bash px4 \
    && echo "px4 ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/px4

USER px4
WORKDIR /home/px4

# Pin the upstream release so every image build produces the same firmware.
# Shallow clone keeps the layer small; HTTP/1.1 avoids stream resets through
# corporate/campus proxies (http_proxy build args are honored by git).
ARG PX4_VERSION=v1.17.0
RUN git -c http.version=HTTP/1.1 clone --depth 1 --branch "$PX4_VERSION" \
        --recursive --shallow-submodules \
        https://github.com/PX4/PX4-Autopilot.git /home/px4/PX4-Autopilot
WORKDIR /home/px4/PX4-Autopilot

RUN sudo bash Tools/setup/ubuntu.sh --no-sim-tools

# Simulation dependencies: the v1.17 script block, with gz-jetty replacing
# the default gz-harmonic (both supported by this firmware version).
RUN --mount=type=cache,target=/var/lib/apt/lists,id=apt_cache_pxsitl,sharing=locked \
    sudo wget -q https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && sudo apt-get update -y \
    && sudo apt-get install -y --no-install-recommends \
        gz-jetty \
        libunwind-dev \
        cppzmq-dev \
        dmidecode \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        libeigen3-dev \
        libgstreamer-plugins-base1.0-dev \
        libimage-exiftool-perl \
        libopencv-dev \
        libxml2-utils \
        pkg-config \
        protobuf-compiler

ENV PATH="/home/px4/PX4-Autopilot/Tools:$PATH"
RUN make px4_sitl_default

RUN echo "export GZ_SIM_RESOURCE_PATH=/home/px4/PX4-Autopilot/Tools/simulation/gz/models:/home/px4/PX4-Autopilot/Tools/simulation/gz/worlds" \
        >> /home/px4/.bashrc

CMD ["bash"]
