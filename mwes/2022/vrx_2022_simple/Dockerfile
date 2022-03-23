FROM ros:noetic-ros-base

# Set Ubuntu release
ARG RELEASE=focal

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8
ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_DRIVER_CAPABILITIES=graphics
ENV NVIDIA_VISIBLE_DEVICES=all

RUN echo 'Etc/UTC' > /etc/timezone \
 && ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
 && apt update \
 && apt install -q -y --no-install-recommends tzdata \
 && rm -rf /var/lib/apt/lists/*

# Required utilities 
RUN apt update \
 && apt install -y --no-install-recommends \
        build-essential \
        dirmngr \
        git \
        gnupg2 \
        python3-rosdep \
        python3-rosinstall \
        python3-vcstools \
 && rm -rf /var/lib/apt/lists/*

RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu ${RELEASE} main" > /etc/apt/sources.list.d/ros1-latest.list' \
 && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && apt update \
 && apt install -q -y \
        ros-noetic-desktop-full=1.5.0-1* \
        ros-noetic-desktop=1.5.0-1* \
        ros-noetic-robot=1.5.0-1* \
        ros-noetic-ros-base=1.5.0-1* \
        ros-noetic-ros-core=1.5.0-1* \
        ros-noetic-geographic-msgs \
 && rm -rf /var/lib/apt/lists/*

# For simplicity, we are cloning the repository directly into the image.
# This is OK because we always want v1.5.0. It is probably not a good
# idea to do this with code that is still being updated, because Docker
# cannot detect remote changes.
RUN mkdir -p /vrx_ws/src \
 && cd vrx_ws/src \
 && git clone -b 1.5.0 https://github.com/osrf/vrx.git

RUN . /opt/ros/noetic/setup.sh \
 && cd /vrx_ws \
 && catkin_make

COPY ros_entrypoint.sh run_my_system.bash basic_node.py /

ENTRYPOINT ["/ros_entrypoint.sh"]
