#!/bin/bash

# run_vrx_task.sh: A shell script to execute one vrx task.
# E.g.: ./run_vrx_task.sh `catkin_find --share osrf_gear`/config/qual3a.yaml
#  `catkin_find --share osrf_gear`/config/sample_user_config.yaml
#  /tmp/team_foo/finalA/1/

set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <task_world> <wamv_urdf> <dst_folder>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 3 ]] && usage

TASK_WORLD=$1
WAMV_URDF=$2
DST_FOLDER=$3

# Create a directory for the Gazebo log and the score file.
if [ -d "$DST_FOLDER" ]; then
  echo -e "${YELLOW}Wrn: Destination folder already exists. Data might be"\
          "overwritten${NOCOLOR}\n"
fi
mkdir -p $DST_FOLDER

echo "Starting vrx task..."

# Run the task.
roslaunch vrx_gazebo wayfinding.launch gui:=false urdf:=$WAMV_URDF world:=$TASK_WORLD &
gazebo_pid=$!

echo -e "${GREEN}OK${NOCOLOR}\n"

echo "Run simulation for 300s before ending"
sleep 300s
echo -e "${GREEN}OK${NOCOLOR}\n"

echo "Killing ${gazebo_pid}"
kill ${gazebo_pid}
