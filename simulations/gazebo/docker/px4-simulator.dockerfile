# PX4 SITL simulator container — stock upstream firmware (no forks).
#
# Mirrors the upstream v1.17 container recipe (ubuntu:24.04 +
# Tools/setup/ubuntu.sh) and installs the SAME Gazebo version the script
# and CI use: Harmonic (gz-sim8). The v1.17 gz_bridge is built and tested
# against Harmonic; newer Gazebo releases (Jetty/gz-sim10) are nominally
# supported but the actuator path was observed broken there. PX4 is pinned
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

# Generous pip retry/timeout: ubuntu.sh installs PX4 python deps and the
# default pip timeouts are too tight behind throttled proxies.
ENV PIP_DEFAULT_TIMEOUT=120 \
    PIP_RETRIES=10

RUN sudo bash Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx

# Simulation dependencies: the v1.17 ubuntu.sh block verbatim (gz-harmonic).
RUN --mount=type=cache,target=/var/lib/apt/lists,id=apt_cache_pxsitl,sharing=locked \
    sudo wget -q https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && sudo apt-get update -y \
    && sudo apt-get install -y --no-install-recommends \
        gz-harmonic \
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
        >> /home/px4/.bashrc \
    && echo "# Offboard-only companion stack: exempt OFFBOARD from the RC and \
datalink loss failsafes (no RC transmitter, GCS heartbeat only)." \
        > /home/px4/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/extras.txt \
    && echo "param set COM_RCL_EXCEPT 12" \
        >> /home/px4/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/extras.txt \
    && echo "param set COM_DLL_EXCEPT 4" \
        >> /home/px4/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/extras.txt \
    && echo "param save" \
        >> /home/px4/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/extras.txt

CMD ["bash"]
