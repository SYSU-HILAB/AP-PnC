FROM px4io/px4-dev-simulation-jammy

RUN apt update && apt install -y \
    vim \
    python3-pip python3-venv \
    curl lsb-release gnupg wget

RUN apt update && apt install -y locales \
    && locale-gen en_US.UTF-8 \
    &&  locale-gen zh_CN.UTF-8 \
    &&  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN apt install -y software-properties-common \
    && add-apt-repository universe \
    && apt update \
    && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb \
    && apt update

RUN apt install -y ros-humble-ros-base

WORKDIR /px4-dependencies
COPY installations/install-dds-agent.sh install-dds-agent.sh
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent \
    && git checkout 7362281
RUN bash install-dds-agent.sh

RUN git clone https://github.com/WarriorHanamy/px4-vtol.git --depth=1 -b docker --recursive \
    /px4-vtol
WORKDIR /px4-vtol
# RUN make px4_sitl_default

# FROM osrf/ros:humble-desktop AS ros-deps

# WORKDIR /plugins
# COPY ../src/aerodynamics plugins/aerodynamics
# COPY ../src/external_libraries plugins/external_libraries
# RUN colcon build 


# Version2 is FROM osrf/ros:humble-desktop to install px4-depencies and gazebo-deps,
# which is very slow, since the downloading speed of gazebo server is only around 20KB/s.
# Both two deps can be install by scripts in <PX4-ROOT>Tools/setup/ubuntu.sh
# Here, we provide in docker/px4-setup to avoid the nuttx toolchain installation.