#!/bin/bash

# run_vrx_trial.sh: A shell script to execute one vrx trial.

set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <wamv_urdf> <trial_world> <destination_folder>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 3 ]] && usage

WAMV_URDF=$1
TRIAL_WORLD=$2
DESTINATION_FOLDER=$3

# Create a directory for the Gazebo log and the score file.
if [ -d "$DESTINATION_FOLDER" ]; then
  echo -e "${YELLOW}Wrn: Destination folder already exists. Data might be"\
          "overwritten${NOCOLOR}\n"
fi
mkdir -p $DESTINATION_FOLDER

## TEMP HACK FIX: to update
cd /home/developer/vrx_ws/src/vrx 
hg update default
cd ../../ && catkin_make
source ./devel/setup.bash

echo "Starting vrx trial..."

# Run the trial.
#roslaunch vrx_gazebo sandisland.launch gui:=false urdf:=$WAMV_URDF world:=$TRIAL_WORLD recording:=true extra_gazebo_args:="--record_path ${DESTINATION_FOLDER}" &
# roslaunch vrx_gazebo sandisland.launch gui:=false urdf:=$WAMV_URDF recording:=true extra_gazebo_args:="--record_path ${DESTINATION_FOLDER}" &
roslaunch vrx_gazebo sandisland.launch gui:=false extra_gazebo_args:="-r" &
roslaunch_pid=$!
echo -e "${GREEN}OK${NOCOLOR}\n"

echo "Starting rosbag recording..." 
sleep 5s
rosbag record -O ~/vrx_task_info.bag /vrx/task/info &
echo -e "${GREEN}OK${NOCOLOR}\n"

echo "Run simulation for 100s before ending"
sleep 100s
echo -e "${GREEN}OK${NOCOLOR}\n"

echo "Killing recorder"
rosnode kill $(rosnode list | grep record | awk '{print $1}')
sleep 2s
echo "Killing roslaunch pid: ${roslaunch_pid}"
kill -INT ${roslaunch_pid}

echo "Finished killing, sleeping for 300s"
sleep 300s
