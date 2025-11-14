FROM px4io/px4-dev-simulation-jammy

RUN apt update && apt install -y software-properties-common \
    && add-apt-repository universe

RUN apt install -y \
    vim \
    python3-pip python3-venv \
    curl lsb-release gnupg wget

RUN apt install -y locales \
    && locale-gen en_US.UTF-8 \
    &&  locale-gen zh_CN.UTF-8 \
    &&  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb

RUN --mount=type=cache,target=/var/lib/apt/lists,id=apt_cache,sharing=locked \
    apt update && \
    apt install -y ros-humble-ros-base

RUN set -ex && \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent_src \
    && cd /tmp/agent_src \
    && git checkout 7362281
COPY installations/install-dds-agent.sh /tmp/agent_src/install-dds-agent.sh
RUN /tmp/agent_src/install-dds-agent.sh

ARG CACHE_BUSTER=default
RUN echo "Cache buster: $CACHE_BUSTER" && \
    git clone https://github.com/WarriorHanamy/px4-vtol.git --depth=1 -b docker --recursive /px4-vtol
WORKDIR /px4-vtol

ENV PATH="/px4-vtol/Tools:$PATH"
RUN make px4_sitl_default

ENTRYPOINT ["runpx4", "2"]

# FROM osrf/ros:humble-desktop AS ros-deps

# WORKDIR /plugins
# COPY ../src/aerodynamics plugins/aerodynamics
# COPY ../src/external_libraries plugins/external_libraries
# RUN colcon build


# Version2 is FROM osrf/ros:humble-desktop to install px4-depencies and gazebo-deps,
# which is very slow, since the downloading speed of gazebo server is only around 20KB/s.
# Both two deps can be install by scripts in <PX4-ROOT>Tools/setup/ubuntu.sh
# Here, we provide in docker/px4-setup to avoid the nuttx toolchain installation.
