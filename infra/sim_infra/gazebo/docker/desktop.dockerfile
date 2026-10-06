FROM ubuntu:24.04
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb x11-utils x11vnc fluxbox novnc websockify xfonts-base fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*
ENV AP_PNC_DIR=/workspace DISPLAY=:99
WORKDIR /workspace
COPY infra/sim_infra/gazebo/docker/desktop-entrypoint.sh /usr/local/bin/desktop-entrypoint
RUN chmod +x /usr/local/bin/desktop-entrypoint
ENTRYPOINT ["/usr/local/bin/desktop-entrypoint"]
