#!/usr/bin/env bash
# This is a bash file that sets up the docker container to run an RL training
# cycle in isolation

set -ex

# cmake compilers and pip
cd /tmp
apt-get update && apt-get install curl build-essential python-dev -y
curl "https://bootstrap.pypa.io/get-pip.py" -o "get-pip.py" && python get-pip.py

# Create the workspace
mkdir -p /catkin_ws/src
cd /catkin_ws/src && catkin_init_workspace && cd ..
ln -s /usr/src/task_sim ./src/

# Install pip requirements
pip install -r src/task_sim/requirements.txt

# Make the workspace and source it
catkin_make
source devel/setup.bash

# Run the launch file
roslaunch task_sim train_rl.launch config_file:="/usr/src/task_sim/$1"
