#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.sh

# Initialize the workspace
catkin init

# Configure the workspace
catkin config --install
