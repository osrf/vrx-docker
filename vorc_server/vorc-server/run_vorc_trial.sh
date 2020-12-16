#!/bin/bash

# run_vorc_trial.sh: A shell script to execute one vorc trial.

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source ${DIR}/gz_utils.sh

set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <task> <trial_world> <destination_folder> [extra_launch_args]"
  echo "  extra_launch_args: If task == gymkhana, specify the absolute path to the YAML file with acoustic pinger position(s) as rosparams, e.g. pinger_params:=<path>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -lt 3 ]] && usage

TASK=$1
TRIAL_WORLD=$2
DESTINATION_FOLDER=$3

# Set the launch file for the given task
LAUNCH_FILE=marina.launch
if [ "$TASK" = "gymkhana" ]; then
  LAUNCH_FILE="gymkhana.launch"
  [[ $# -lt 4 ]] && usage
  EXTRA_LAUNCH_ARGS=$4
elif [ "$TASK" = "perception" ]; then
  LAUNCH_FILE="perception.launch"
elif [ "$TASK" = "stationkeeping" ]; then
  LAUNCH_FILE="station_keeping.launch"
elif [ "$TASK" = "wayfinding" ]; then
  LAUNCH_FILE="wayfinding.launch"
else
  echo "Undefined task $TASK. Check your task name."
  exit 1
fi

# Create a directory for the Gazebo log and the score file.
if [ -d "$DESTINATION_FOLDER" ]; then
  echo -e "${YELLOW}Wrn: Destination folder already exists. Data might be"\
          "overwritten${NOCOLOR}\n"
fi
mkdir -p $DESTINATION_FOLDER

# Sanity check if file exists
if [ -f "${TRIAL_WORLD}" ]; then
  echo "Successfully found: ${TRIAL_WORLD}"
else
  echo -e "${RED}Err: ${TRIAL_WORLD} not found."; ls /task_generated; exit 1;
fi

echo "Starting vorc trial..."

# Run the trial.
# Note: Increase record period to have faster playback. Decrease record period for slower playback
RECORD_PERIOD="0.01"
# To watch the actual competition run, set gui:=true.
roslaunch vorc_gazebo ${LAUNCH_FILE} ${EXTRA_LAUNCH_ARGS} gui:=true world:=$TRIAL_WORLD extra_gazebo_args:="-r --record_period ${RECORD_PERIOD} --record_path $HOME/.gazebo" verbose:=true robot_locked:=true non_competition_mode:=false > $HOME/verbose_output.txt 2>&1 &
roslaunch_pid=$!
wait_until_gzserver_is_up
echo -e "${GREEN}OK${NOCOLOR}\n"

# Store topics in rosbag 
# July 24, 2019 only record task info to save space
echo "Starting rosbag recording..." 
rosbag record -O $HOME/vorc_rostopics.bag /vorc/task/info &
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
