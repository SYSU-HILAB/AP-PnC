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

# Create px4 user with UID 1001 (avoiding conflict with qgc's 1000)
RUN groupadd -r px4 && useradd -m -g px4 -u 1001 -s /bin/bash px4

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb

RUN --mount=type=cache,target=/var/lib/apt/lists,id=apt_cache,sharing=locked \
    apt update && \
    apt install -y ros-humble-ros-base ros-dev-tools

RUN set -ex && \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent_src \
    && cd /tmp/agent_src \
    && git checkout 7362281
COPY installations/install-dds-agent.sh /tmp/agent_src/install-dds-agent.sh
RUN /tmp/agent_src/install-dds-agent.sh

ARG CACHE_BUSTER="rebuild"
RUN echo "Cache buster: $CACHE_BUSTER" && \
    git clone https://github.com/WarriorHanamy/px4-vtol.git --recursive /home/px4/px4-vtol
WORKDIR /home/px4/px4-vtol

ENV PATH="/home/px4/px4-vtol/Tools:$PATH"
RUN make px4_sitl_default

COPY ./src/aerodynamics /plugins/aerodynamics
COPY ./src/utils /plugins/utils
COPY ./src/external_libraries /plugins/external_libraries

RUN echo "source /opt/ros/humble/setup.bash" >> /home/px4/.bashrc
ENV AERO_SIM_DATA_DIR=/plugins/aerodynamics/data

# Give px4 user ownership of their home directory and installed files
RUN chown -R px4:px4 /home/px4 /opt/ros /plugins

# Switch to px4 user
USER px4
WORKDIR /home/px4


#WORKDIR /plugins
#RUN . /opt/ros/humble/setup.sh \
#    && colcon build --symlink-install

#WORKDIR /px4-vtol
ENTRYPOINT ["runpx4", "1"]
