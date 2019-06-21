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
  echo "Usage: $0 <task.yaml> <user.yaml> <dst_folder>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 3 ]] && usage

TASK_CONFIG=$1
USER_CONFIG=$2
DST_FOLDER=$3

# Create a directory for the Gazebo log and the score file.
if [ -d "$DST_FOLDER" ]; then
  echo -e "${YELLOW}Wrn: Destination folder already exists. Data might be"\
          "overwritten${NOCOLOR}"
fi
mkdir -p $DST_FOLDER

echo "Running vrx task..."

# Run the task.
vrx_EXIT_ON_COMPLETION=1 roslaunch vrx_gazebo sandisland.launch gui:=false 

echo -e "${GREEN}OK${NOCOLOR}"

echo "Sleeping for 10s"
sleep 5s

rostopic echo /vrx/task/info

# Copy vrx log files.
echo -n "Copying logs into [$DST_FOLDER]..."
cp --recursive --dereference ~/.vrx/log/* $DST_FOLDER
# Copy ROS log files.
mkdir -p $DST_FOLDER/ros
cp --recursive --dereference ~/.ros/log/latest/* $DST_FOLDER/ros
# Copy vrx generated files.
mkdir -p $DST_FOLDER/generated
cp --recursive --dereference /tmp/vrx/* $DST_FOLDER/generated

echo -e "${GREEN}OK${NOCOLOR}"
