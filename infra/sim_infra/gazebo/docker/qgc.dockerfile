# syntax=docker/dockerfile:1
# QGC is independent of ROS; upstream osrf/ros desktop is not multiarch.
FROM ubuntu:24.04
ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH
ENV AP_PNC_DIR=/workspace
WORKDIR /workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl file libgl1 libegl1 libopengl0 libgl1-mesa-dri \
    libxcb-xinerama0 libxcb-cursor0 libxcb-icccm4 libxcb-keysyms1 libxcb-render-util0 \
    libxkbcommon-x11-0 libnss3 libasound2t64 libpulse0 libspeechd2 fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*
# AppImage bundles Qt, but these system font/X session libraries are still needed.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libfontconfig1 libfreetype6 libsm6 libice6 \
    && rm -rf /var/lib/apt/lists/*
# Version and BOTH hashes must be updated together; fail closed on unsupported targets.
RUN case "$TARGETARCH" in \
    arm64) asset=aarch64; sha=ddd92fe3a1cd1c1c21a3028052e9f9b4d4a68cb80689552d3f34727ed2c91e03 ;; \
    amd64) asset=x86_64; sha=9a47e4cf269d9e4f897f582f6b19aa7ebf1e9e67c9d68a532a45b541354bc344 ;; \
    *) echo "Unsupported QGC target: $TARGETARCH" >&2; exit 2 ;; \
    esac \
    && mkdir -p "$AP_PNC_DIR/.artifacts/qgc" \
    && cd "$AP_PNC_DIR/.artifacts/qgc" \
    && curl -fL --retry 5 "https://github.com/mavlink/qgroundcontrol/releases/download/v5.1.5/QGroundControl-${asset}.AppImage" -o package.AppImage \
    && echo "$sha  package.AppImage" | sha256sum -c - \
    && chmod +x "$AP_PNC_DIR/.artifacts/qgc/package.AppImage" \
    && "$AP_PNC_DIR/.artifacts/qgc/package.AppImage" --appimage-extract \
    && chmod -R a+rX "$AP_PNC_DIR/.artifacts/qgc/squashfs-root" \
    && rm "$AP_PNC_DIR/.artifacts/qgc/package.AppImage" \
    && useradd -m -u 1003 qgc \
    && mkdir -p "$AP_PNC_DIR/.artifacts/qgc_home" \
    && chown qgc:qgc "$AP_PNC_DIR/.artifacts/qgc_home"
USER qgc
ENV HOME=/workspace/.artifacts/qgc_home
ENTRYPOINT ["/workspace/.artifacts/qgc/squashfs-root/AppRun"]
