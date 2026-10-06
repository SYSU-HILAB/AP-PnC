# syntax=docker/dockerfile:1
# Stock upstream PX4, native ARM64/AMD64; no host GPU or X server required.
FROM ubuntu:24.04
ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH
ARG PX4_VERSION=v1.17.0
ENV AP_PNC_DIR=/workspace
WORKDIR /workspace

RUN --mount=type=cache,target=/var/lib/apt/lists,id=ap-pnc-px4-${TARGETARCH},sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl git gnupg build-essential cmake ninja-build pkg-config \
    python3-dev python3-pip python3-venv python3-setuptools \
    libunwind-dev cppzmq-dev libeigen3-dev protobuf-compiler libxml2-utils \
    gosu procps \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/gazebo.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
        > /etc/apt/sources.list.d/gazebo.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    gz-sim8-cli libgz-sim8-plugins libgz-sim8-dev gz-transport13-cli \
    libgz-physics7-dartsim libgz-physics7-dartsim-dev libgz-rendering8-ogre2 \
    libgl1-mesa-dri libopencv-dev libgz-rendering8-core-dev libgz-rendering8-ogre2-dev \
    libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    qml-module-qtquick-controls qml-module-qtquick-controls2 qml-module-qtquick-layouts \
    qml-module-qtquick-window2 qml-module-qtquick2 qml-module-qtgraphicaleffects \
    qml-module-qtquick-dialogs qml-module-qt-labs-platform qml-module-qt-labs-folderlistmodel \
    qml-module-qt-labs-settings libqt5svg5

RUN useradd -m -u 1002 -s /bin/bash px4 \
    && mkdir -p "$AP_PNC_DIR/.artifacts" \
    && chown px4:px4 "$AP_PNC_DIR/.artifacts"
USER px4
RUN git -c http.version=HTTP/1.1 clone --depth 1 --branch "$PX4_VERSION" \
    https://github.com/PX4/PX4-Autopilot.git "$AP_PNC_DIR/.artifacts/px4_src" \
    && git -C "$AP_PNC_DIR/.artifacts/px4_src" -c http.version=HTTP/1.1 \
       submodule update --init --recursive --depth 1 \
       src/modules/mavlink/mavlink src/drivers/gps/devices src/lib/events/libevents \
       src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client \
       src/lib/cdrstream/cyclonedds src/lib/cdrstream/rosidl src/lib/heatshrink/heatshrink \
       Tools/simulation/gz
RUN python3 -m venv "$AP_PNC_DIR/.artifacts/px4_venv" \
    && "$AP_PNC_DIR/.artifacts/px4_venv/bin/pip" install --no-cache-dir \
       -r "$AP_PNC_DIR/.artifacts/px4_src/Tools/setup/requirements.txt" pymavlink==2.4.49
ENV PATH="/workspace/.artifacts/px4_venv/bin:$PATH"
WORKDIR /workspace/.artifacts/px4_src
ARG BUILD_JOBS=2
RUN make px4_sitl_default -j"$BUILD_JOBS"
RUN printf '%s\n' 'param set COM_RCL_EXCEPT 12' 'param set COM_DLL_EXCEPT 4' 'param save' \
    > "$AP_PNC_DIR/.artifacts/px4_src/build/px4_sitl_default/rootfs/etc/extras.txt"

USER root
COPY infra/sim_infra/gazebo/docker/px4-entrypoint.sh /usr/local/bin/px4-entrypoint
COPY infra/sim_infra/gazebo/scripts/gcs_heartbeat.py /workspace/infra/sim_infra/gazebo/scripts/gcs_heartbeat.py
RUN chmod +x /usr/local/bin/px4-entrypoint
ENV GZ_IP=127.0.0.1 GZ_PARTITION=ap_pnc_sitl HEADLESS=1
ENTRYPOINT ["/usr/local/bin/px4-entrypoint"]
CMD ["server"]
