#!/usr/bin/env bash
# Installs a ROS Workspace that can be used in the image

set -ex

# Setup and build the ROS workspace
source "/opt/ros/$ROS_DISTRO/setup.bash"
mkdir -p $TASKSIM_FILES_ROOT/workspace/src
cd $TASKSIM_FILES_ROOT/workspace/src && catkin_init_workspace && cd ..
catkin_make
