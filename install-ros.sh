#!/usr/bin/bash
if [ -f "Miniconda3-latest-Linux-x86_64.sh" ]; then
	  echo "Miniconda3-latest-Linux-x86_64.sh exists!"
  else
	    echo "Installing Miniconda3-latest-Linux-x86_64.sh"
	      curl -O https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
fi

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings


sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
sudo apt install ros-humble-plotju*
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-vrpn
sudo apt install ccache fzf cloc
sudo apt install python3-virtualenv
sudo apt install ros-humble-ros-gz-*
sudo apt install ros-humble-ros-gz