#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.sh

rosdep install --from-paths src --default-yes --ignore-src
