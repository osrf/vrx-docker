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
  echo "Usage: $0 <wamv_urdf> <task_world> <dst_folder>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 3 ]] && usage

WAMV_URDF=$1
TASK_WORLD=$2
DST_FOLDER=$3

# Create a directory for the Gazebo log and the score file.
if [ -d "$DST_FOLDER" ]; then
  echo -e "${YELLOW}Wrn: Destination folder already exists. Data might be"\
          "overwritten${NOCOLOR}\n"
fi
mkdir -p $DST_FOLDER

## TEMP HACK FIX: to update
cd /home/developer/vrx_ws/src/vrx 
hg update default
cd ../../ && catkin_make
source ./devel/setup.bash

echo "Starting vrx task..."

# Run the task.
#roslaunch vrx_gazebo sandisland.launch gui:=false urdf:=$WAMV_URDF world:=$TASK_WORLD recording:=true extra_gazebo_args:="--record_path ${DST_FOLDER}" &
# roslaunch vrx_gazebo sandisland.launch gui:=false urdf:=$WAMV_URDF recording:=true extra_gazebo_args:="--record_path ${DST_FOLDER}" &
roslaunch vrx_gazebo sandisland.launch gui:=false extra_gazebo_args:="-r" &
gazebo_pid=$!

echo -e "${GREEN}OK${NOCOLOR}\n"

echo "Run simulation for 100s before ending"
sleep 100s
echo -e "${GREEN}OK${NOCOLOR}\n"

echo "Killing ${gazebo_pid}"
kill ${gazebo_pid}
echo "Killed Gazebo, sleeping for 300s"
sleep 300s
