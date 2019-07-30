#!/bin/bash

# run_vrx_trial.sh: A shell script to execute one vrx trial.

is_gzserver_running()
{
  if pgrep gzserver >/dev/null; then
    true
  else
    false
  fi
}

wait_until_gzserver_is_down()
{
  until ! is_gzserver_running
  do
    sleep 1
  done
}


wait_until_gzserver_is_up()
{
  until is_gzserver_running
  do
    sleep 1
  done
}

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

echo "Starting vrx trial..."

# Run the trial.
RECORD_PERIOD="0.01"
roslaunch vrx_gazebo sandisland.launch gui:=false urdf:=$WAMV_URDF world:=$TRIAL_WORLD extra_gazebo_args:="-r --record_period ${RECORD_PERIOD} --record_path $HOME/.gazebo" verbose:=true > ~/verbose_output.txt 2>&1 &
roslaunch_pid=$!
wait_until_gzserver_is_up
echo -e "${GREEN}OK${NOCOLOR}\n"

# Store topics in rosbag NOTE: currently storing ALL topics, might be too much
# July 24, 2019 only record task info to save space
echo "Starting rosbag recording..." 
rosbag record -O ~/vrx_rostopics.bag /vrx/task/info &
echo -e "${GREEN}OK${NOCOLOR}\n"

# Run simulation until shutdown
echo "Run simulation until gzserver is shutdown by scoring plugin"
wait_until_gzserver_is_down
echo "gzserver shut down"
echo -e "${GREEN}OK${NOCOLOR}\n"

# Kill rosnodes
echo "Killing rosnodes"
rosnode kill --all
sleep 1s

# Kill roslaunch
echo "Killing roslaunch pid: ${roslaunch_pid}"
kill -INT ${roslaunch_pid}
echo -e "${GREEN}OK${NOCOLOR}\n"
