FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    vim python3-pip python3-venv curl wget \
    ros-dev-tools ccache fzf cloc \
    ros-humble-vrpn ros-humble-plotjuggler* libceres-dev

WORKDIR /ap-pnc

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /usr/local/bin/

COPY pyproject.toml uv.lock ./
RUN uv sync --group dev

COPY installations/custom_funcs.sh /tmp/custom_funcs.sh
RUN cat /tmp/custom_funcs.sh >> /root/.bashrc && rm /tmp/custom_funcs.sh

COPY . .

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
    --packages-ignore aerodynamics \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
