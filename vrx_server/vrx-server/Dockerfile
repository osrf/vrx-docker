ARG BASEIMG=ubuntu:jammy
FROM $BASEIMG

# Set ROS distribution
ARG ROSDIST=humble

# Set Gazebo verison
ARG GZDIST=garden

ENV GZ_VERSION garden

RUN export DEBIAN_FRONTEND=noninteractive \
 && apt update \
 && apt install -y \
    tzdata \
 && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt clean

# Tools useful during development.
 RUN apt update \
  && apt install -y \
         build-essential \
         cmake \
         cppcheck \
         curl \
         git \
         language-pack-en \
         libeigen3-dev \
         libgles2-mesa-dev \
         lsb-release \
         gdb \
         mercurial \
         pkg-config \
         psmisc \
         python3-dbg \
         python3-pip \
         python3-venv \
         qtbase5-dev \
         ruby \
         software-properties-common \
         sudo \
         vim \
         wget \
         libgflags-dev \
  && apt clean



# Get ROS and Gazebo
# Install ignition fortress and ROS2 Desktop
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null' \
  && /bin/sh -c 'curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null' \
  && apt update \
  && apt install -y --no-install-recommends \
     gz-${GZDIST} \
     ros-${ROSDIST}-desktop \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean -qq

RUN apt update \
 && apt install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-sdformat13 \
    ros-${ROSDIST}-compressed-image-transport \
    ros-${ROSDIST}-joy \
    ros-${ROSDIST}-joy-teleop \
    ros-${ROSDIST}-key-teleop \
    ros-${ROSDIST}-robot-localization \
    ros-${ROSDIST}-robot-state-publisher \
    ros-${ROSDIST}-rviz2 \
    ros-${ROSDIST}-teleop-tools \
    ros-${ROSDIST}-teleop-twist-keyboard \
    ros-${ROSDIST}-xacro \
 && apt clean

# Removed
#    python-rosdep \
# && rosdep init \
# RUN rosdep update

# Set USER and GROUP
ARG USER=developer
ARG GROUP=developer
ARG USERID=1000
ARG GROUPID=1000

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`.

RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

RUN addgroup --gid $GROUPID $USER && \
    useradd --no-log-init --create-home --uid $USERID --gid $GROUPID --home-dir /home/$USER --shell /bin/sh -c "" $USER

RUN adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# Commands below run as the developer user.
USER $USER:$GROUP

# When running a container start in the developer's home folder.
WORKDIR /home/$USER

# Create workspace
RUN mkdir -p vrx_ws/src

# Copy the VRX repository from the local file system
# We can't use the USER:GROUP variables until Docker adds support to --chown
# COPY --chown=developer:developer . vrx_ws/src/vrx/
RUN git clone -b 2.0.0 https://github.com/osrf/vrx.git \
&& git clone https://github.com/gazebosim/ros_gz.git -b humble \
&& mv ./vrx ./vrx_ws/src \
&& mv ./ros_gz ./vrx_ws/src

# Compile the VRX project.
RUN /bin/bash -c ". /opt/ros/${ROSDIST}/setup.bash && cd vrx_ws && GZ_VERSION=garden colcon build --merge-install"

# Source all the needed environment files.
RUN /bin/sh -c 'echo ". /opt/ros/${DIST}/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". ~/vrx_ws/devel/setup.sh" >> ~/.bashrc'
## END OF SECTION BASED ON vrx/docker/Dockerfile

# Expose port used to communiate with gzserver
EXPOSE 11345

# Stuff for nvidia-docker
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# Set encoding to use unicode characters
# RUN sudo locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# setup entrypoint
COPY ./vrx_entrypoint.sh /
COPY ./run_vrx_trial.sh /
COPY ./play_vrx_log.sh /
COPY ./gz_utils.sh /

ENTRYPOINT ["/vrx_entrypoint.sh"]
