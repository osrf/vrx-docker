FROM ros:noetic-ros-base

# Set Ubuntu release
ARG RELEASE=focal
ARG DIST=noetic

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8
ENV ROS_DISTRO=${DIST}
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_DRIVER_CAPABILITIES=graphics
ENV NVIDIA_VISIBLE_DEVICES=all

# Setting the TimeZone
RUN echo 'Etc/UTC' > /etc/timezone \
 && ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
 && apt update \
 && apt install -q -y --no-install-recommends \
    tzdata \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt clean \
 && rm -rf /var/lib/apt/lists/*

# Required utilities
RUN apt update \
 && apt install -y --no-install-recommends\
        build-essential \
        cmake \
        cppcheck \
        curl \
        git \
        gnupg \
        libeigen3-dev \
        libgles2-mesa-dev \
        lsb-release \
        pkg-config \
        protobuf-compiler \
        python3-dbg \
        python3-pip \
        python3-scipy \
        python3-venv \
        qtbase5-dev \
        ruby \
        software-properties-common \
        sudo \
        wget \
 && apt clean

# Get whatever packages you need to build your software
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu ${RELEASE} main" > /etc/apt/sources.list.d/ros1-latest.list' \
 && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && apt update \
 && apt install -y --no-install-recommends \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    ros-${DIST}-effort-controllers \
    ros-${DIST}-geographic-info \
    ros-${DIST}-joint-state-controller \
    ros-${DIST}-joint-state-publisher \
    ros-${DIST}-joy \
    ros-${DIST}-joy-teleop \
    ros-${DIST}-kdl-parser-py \
    ros-${DIST}-key-teleop \
    ros-${DIST}-move-base \
    ros-${DIST}-robot-localization \
    ros-${DIST}-robot-state-publisher \
    ros-${DIST}-ros-base \
    ros-${DIST}-ros-controllers \
    ros-${DIST}-rqt \
    ros-${DIST}-rqt-common-plugins \
    ros-${DIST}-rqt-robot-plugins \
    ros-${DIST}-teleop-tools \
    ros-${DIST}-teleop-twist-joy \
    ros-${DIST}-teleop-twist-keyboard \
    ros-${DIST}-tf2-geometry-msgs \
    ros-${DIST}-tf2-tools \
    ros-${DIST}-urdfdom-py \
    ros-${DIST}-velodyne-simulator \
    ros-${DIST}-xacro \
 && apt clean

# Optional: Dev. tools, applications, etc.
RUN apt-get update && apt-get install -y --no-install-recommends \
    gdb \
    psmisc \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Set up a catkin workspace
RUN mkdir -p /vrx_ws/src

# Here we use my_vrx as an example of a source code repository stored 
# locally on the host. Depending on your solution, you may not 
# necessarily need to build vrx. Modify the line below to indicate
# the source repository you want to copy into the workspace inside your
# image.
COPY my_vrx /vrx_ws/src/vrx

# By default, environment variables do not persist from one Docker
# command to the next, so it is necessary to source setup.sh and build
# the workspace in the same command.
RUN . /opt/ros/noetic/setup.sh \
 && cd /vrx_ws \
 && catkin_make

# Copy startup scripts from host filesystem
COPY ros_entrypoint.sh run_my_system.bash basic_node.py /

## Customize your image here.
#
#
## ...

# Sets the entrypoint to your `/ros_entrypoint.sh` script. This is the default
# for this image unless you explicitly change it (not usually necessary).
ENTRYPOINT ["/ros_entrypoint.sh"]
