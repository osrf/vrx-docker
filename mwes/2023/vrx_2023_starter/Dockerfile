# Note: you must first build the `dockwater:humble` image locally as described
# in the README
ARG BASEIMG=dockwater:humble
FROM $BASEIMG 

# Set Ubuntu release
ARG ROSDIST=humble
ARG GZDIST=garden

# Optional: Uncomment and customize to get whatever packages you need to build 
# your software
#RUN apt update \
#&& apt install -y --no-install-recommends \
#    ros-${ROSDIST}-actuator-msgs \
#&& rm -rf /var/lib/apt/lists/* \
#&& apt clean -qq

# Optional: Uncomment and customize to add dev. tools, applications, etc. 
#RUN apt-get update && apt-get install -y --no-install-recommends \
#    gdb \
#    psmisc \
#    vim \
#    && rm -rf /var/lib/apt/lists/*

# Set up a colcon workspace
RUN mkdir -p /vrx_ws/src

# Here we use my_vrx as an example of a source code repository stored 
# locally on the host. Depending on your solution, you may not 
# necessarily need to build vrx. Modify the line below to indicate
# the source repository you want to copy into the workspace inside your
# image.
COPY my_source /vrx_ws/src/vrx

# By default, environment variables do not persist from one Docker
# command to the next, so it is necessary to source setup.sh and build
# the workspace in the same command.
RUN . /opt/ros/humble/setup.sh \
 && cd /vrx_ws \
 && colcon build --merge-install

# Copy startup scripts from host filesystem
COPY ros_entrypoint.sh run_my_system.bash wamv_demo.py /

## Customize your image here.
#
#
## ...

# Sets the entrypoint to your `/ros_entrypoint.sh` script. This is the default
# for this image unless you explicitly change it (not usually necessary).
ENTRYPOINT ["/ros_entrypoint.sh"]
