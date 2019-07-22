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

## Get latest repository and compile
cd /home/developer/vrx_ws/src/vrx 
hg pull
hg update default
cd ../../ && catkin_make
source ./devel/setup.bash

echo "Starting vrx trial..."

# Run the trial.
RECORD_PERIOD="0.05"
roslaunch vrx_gazebo sandisland.launch gui:=false urdf:=$WAMV_URDF world:=$TRIAL_WORLD extra_gazebo_args:="-r --record_period ${RECORD_PERIOD}" verbose:=true &
# roslaunch vrx_gazebo sandisland.launch gui:=false urdf:=$WAMV_URDF extra_gazebo_args:="-r --record_period ${RECORD_PERIOD}" verbose:=true &
# roslaunch vrx_gazebo sandisland.launch gui:=false extra_gazebo_args:="-r --record_period ${RECORD_PERIOD}" verbose:=true &
roslaunch_pid=$!
echo -e "${GREEN}OK${NOCOLOR}\n"

# Store topics in rosbag NOTE: currently storing ALL topics, might be too much
echo "Starting rosbag recording..." 
sleep 5s
rosbag record -O ~/vrx_task_info.bag --all &
echo -e "${GREEN}OK${NOCOLOR}\n"

# Let simulation run
echo "Run simulation for 100s before ending"
sleep 100s
echo -e "${GREEN}OK${NOCOLOR}\n"

# Kill rosbag record
echo "Killing recorder"
rosnode kill $(rosnode list | grep record | awk '{print $1}')
sleep 2s

# Kill roslaunch
echo "Killing roslaunch pid: ${roslaunch_pid}"
kill -INT ${roslaunch_pid}

echo "Finished killing, sleeping for 300s"
sleep 300s