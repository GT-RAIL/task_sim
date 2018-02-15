# Create the image to perform RL task learning

FROM nvidia/cuda:8.0-cudnn5-devel-ubuntu14.04
ARG TASKSIM_FILES_ROOT
ENV TASKSIM_FILES_ROOT ${TASKSIM_FILES_ROOT:-/root}

# Prepare to do installation in the /tmp directory
WORKDIR /tmp
COPY requirements.txt bin/docker/rl/ros_init.sh ./
COPY bin/docker/rl/ros_entrypoint.sh $TASKSIM_FILES_ROOT/

# install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    curl \
    build-essential \
    libhdf5-dev

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO indigo
RUN apt-get install -y \
    ros-indigo-ros-core=1.1.5-0* \
    ros-indigo-ros-base=1.1.5-0* \
    && rm -rf /var/lib/apt/lists/*

# Install pip and the other python dependencies
RUN curl "https://bootstrap.pypa.io/get-pip.py" -o "get-pip.py" \
    && python get-pip.py \
    && pip install -r requirements.txt \
    && pip install http://download.pytorch.org/whl/cu80/torch-0.3.1-cp27-cp27mu-linux_x86_64.whl \
    && pip install torchvision

# Make the catkin_ws, build it and source it
RUN ./ros_init.sh

WORKDIR $TASKSIM_FILES_ROOT/workspace
ENTRYPOINT ["../ros_entrypoint.sh"]
CMD ["bash"]
