#!/bin/bash

# This file configures finishes the configuration of Ubuntu 16.04 for the MIE443
# course.

# Install ROS as per the official instructions found at:
# http://wiki.ros.org/kinetic/Installation/Ubuntu
echo "Setting up ROS"

sudo add-apt-repository restricted
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update -y
sudo apt-get install ros-kinetic-desktop-full -y

sudo rosdep init
rosdep update
source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y

# Set up the catkin workspace
echo "Setting up catkin workspace"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install turtlebot packages
echo "Setting up turtlebot"
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs -y
rosrun kobuki_ftdi create_udev_rules
echo "export TURTLEBOT_3D_SENSOR=kinect" >> ~/.bashrc

# Instal Visual Studio Code as per the official instructions found at:
# https://code.visualstudio.com/docs/setup/linux
cd ~
curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/
sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt-get install apt-transport-https -y
sudo apt-get update -y
sudo apt-get install code -y


# Install some usefull Visual Studio Code extensions
echo "Setting up VS Code extensions"
# c++ linter,debugging, syntax highlight etc...
code --install-extension ms-vscode.cpptools
# remote development tool
code --install-extension ms-vscode-remote.vscode-remote-extensionpack
# c++ code snippets
code --install-extension hars.CppSnippets
# bracket color to indicate multiple brackets
code --install-extension CoenraadS.bracket-pair-colorizer
# ros msg syntax highlight
code --install-extension ajshort.msg
# yaml file syntax highlight
code --install-extension redhat.vscode-yaml
# cmake file syntax highlight
code --install-extension twxs.cmake
# for python equivalent intellisense/debug/syntax highlight etc
code --install-extension  ms-python.python
# for ROS
code --install-extension ms-iot.vscode-ros

# Install additional packages
sudo apt-get install terminator -y
sudo apt-get install ros-kinetic-audio-common -y

# Apply updates
sudo apt-get upgrade -y

# Finishing 
source ~/.bashrc
