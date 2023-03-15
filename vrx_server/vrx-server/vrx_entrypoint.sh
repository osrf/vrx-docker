#!/bin/bash
set -e

# first, execute overriden entrypoint from gazebo
# source "/usr/share/gazebo/setup.sh"

# setup ros environment.
source "/opt/ros/humble/setup.bash" > /dev/null

# setup vrx environment
source /home/developer/vrx_ws/devel/setup.sh
echo "vrx entrypoint executed"

# TODO: optionally disable this so a gzclient can be run on the host for development.
export GAZEBO_IP=127.0.0.1
export GAZEBO_IP_WHITE_LIST=127.0.0.1

exec "$@"
