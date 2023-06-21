ARG BASEIMG=npslearninglab/watery_robots:vrx_current
FROM $BASEIMG

# Set ROS distribution
ARG ROSDIST=humble

# Create workspace
RUN mkdir -p ~/vrx_ws/src

# TODO: restore version tag
RUN git clone https://github.com/osrf/vrx.git \
&& mv ./vrx ./vrx_ws/src

# Compile the VRX project.
RUN /bin/bash -c ". /opt/ros/${ROSDIST}/setup.bash && cd vrx_ws && colcon build --merge-install"

# Source all the needed environment files.
RUN /bin/sh -c 'echo ". /opt/ros/${ROSDIST}/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". ~/vrx_ws/install/setup.sh" >> ~/.bashrc'
## END OF SECTION BASED ON vrx/docker/Dockerfile

# Expose port used to communiate with gzserver
EXPOSE 11345

# setup entrypoint
COPY ./vrx_entrypoint.sh /
COPY ./run_vrx_trial.sh /
COPY ./play_vrx_log.sh /
COPY ./gz_utils.sh /

ENTRYPOINT ["/vrx_entrypoint.sh"]
