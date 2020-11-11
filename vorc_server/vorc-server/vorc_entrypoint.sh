#!/bin/bash
set -e

# first, execute overriden entrypoint from gazebo
source "/usr/share/gazebo/setup.sh"

# setup ros environment.
source "/opt/ros/melodic/setup.bash" > /dev/null

# setup vorc environment
# TODO(mabelzhang): change to install after fixing vrx package to work with install
# source ~/vorc_ws/install/setup.bash
source ~/vorc_ws/devel/setup.bash
echo "vorc entrypoint executed"

# TODO: optionally disable this so a gzclient can be run on the host for development.
export GAZEBO_IP=127.0.0.1
export GAZEBO_IP_WHITE_LIST=127.0.0.1

exec "$@"
