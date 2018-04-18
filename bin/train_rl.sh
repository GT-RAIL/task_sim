#!/usr/bin/env bash
# This is a bash file that sets up the docker container to run an RL training
# cycle in isolation

set -ex

# Recreate and build the workspace
cd $TASKSIM_FILES_ROOT/workspace
ln -s $TASKSIM_FILES_ROOT/software/task_sim ./src/
catkin_make
source devel/setup.bash

# Launch the training script
roslaunch task_sim train_rl.launch config_file:="$TASKSIM_FILES_ROOT/software/task_sim/$1"
