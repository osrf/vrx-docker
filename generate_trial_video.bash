#!/bin/bash

# generate_trial_video.bash: A bash script to generate a video from a Gazebo playback log file.
#
# E.g.: ./generate_trial_video.bash example_team station_keeping 0
#
# Please, install the following dependencies before using the script:
#   sudo apt-get install recordmydesktop wmctrl psmisc

# Exit on error
set -e

# Constants
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <team_name> <task_name> <trial_num>"
  exit 1
}

# Check if gzclient is running
is_gzclient_running()
{
  if pgrep gzclient >/dev/null; then
    true
  else
    false
  fi
}

# Wait until the gazebo world stats topic (eg. /gazebo/robotx_example_course/world_stats /gazebo/<world>/world_stats)
# tells us that the playback has been paused. This event will trigger the end of the recording.
wait_until_playback_ends()
{
  echo -n "Waiting for playback to end..."
  gz_world_stats_topic=$(gz topic -l | grep "world_stats")

  until gz topic -e $gz_world_stats_topic -d 1 -u | grep "paused: true" \
    > /dev/null
  do
    sleep 1
    if ! is_gzclient_running ; then
      echo 1>&2 "GZ client not running, bailing"
      return 0
    fi
  done
  echo -e "${GREEN}OK${NOCOLOR}"
}

# Call usage() function if arguments not supplied.
[[ $# -ne 3 ]] && usage

TEAM_NAME=$1
TASK_NAME=$2
TRIAL_NUM=$3

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

GZ_LOG_FILE=$DIR/generated/logs/$TEAM_NAME/$TASK_NAME/$TRIAL_NUM/gazebo-server/state.log

# Output directory
OUTPUT_DIR=$DIR/generated/logs/$TEAM_NAME/$TASK_NAME/$TRIAL_NUM/video
if [ -d "$OUTPUT_DIR" ]; then
  echo "Overwriting directory: ${OUTPUT_DIR}"
  rm -R $OUTPUT_DIR
else
  echo "Creating directory: ${OUTPUT_DIR}"
fi

mkdir -p $OUTPUT_DIR
OUTPUT=$OUTPUT_DIR/playback_video.ogv

# Sanity check: Make sure that the log file exists.
if [ ! -f $GZ_LOG_FILE ]; then
    echo "Gazebo log file [$GZ_LOG_FILE] not found!"
    exit 1
else
    echo "Found Gazebo log file [$GZ_LOG_FILE]"
fi

# Sanity check: Make sure that catkin_find is found.
which catkin_find > /dev/null || { echo "Unable to find catkin_find."\
  "Did you source your ROS setup.bash file?" ; exit 1; }

# Sanity check: Kill any dangling Gazebo before moving forward.
killall -wq gzserver gzclient || true

echo "Sanity checks complete"

# Define constants for recording
x=100
y=100
width=1000
height=750
BLACK_WINDOW_TIME=2

# Tell gazebo client what size and place it should be
echo "[geometry]
width=$width
height=$height
x=$x
y=$y" > ~/.gazebo/gui.ini

# Start Gazebo in playback mode
roslaunch vrx_gazebo playback.launch log_file:=$GZ_LOG_FILE paused:=true verbose:=true \
  > $OUTPUT.playback_output.txt 2>&1 &
sleep 1s

# Check if the log file has errors, likely forgot to source ws
if grep -Fq "RLException" $OUTPUT.playback_output.txt
then
  echo "Failed to find playback launch file. Did you source your vrx_ws?"
  exit 1
fi

echo "Setting up for playback..."

# Wait and find the Gazebo Window ID.
# Note: May find Gazebo in browser tab, which is wrong. Please close all Gazebo related tabs.
until wmctrl -lp | grep Gazebo > /dev/null
do
  sleep 1
done
GAZEBO_WINDOW_ID=`wmctrl -lp | grep Gazebo | cut -d" " -f 1`

if [ -z "$GAZEBO_WINDOW_ID" ]; then
  echo "Gazebo window not detected. Exiting..."
  sleep 2
  killall -w gzserver gzclient
  exit 1
fi

# Move Gazebo window to front
wmctrl -i -a ${GAZEBO_WINDOW_ID}

# Adjust the value of this constant if needed to avoid capturing a black
# screen for a long time.
sleep $BLACK_WINDOW_TIME

# Unpause the simulation.
echo -n "Playing back..."
gz world -p 0
echo -e "${GREEN}OK${NOCOLOR}"

# Start recording the Gazebo Window.
echo -n "Recording..."
recordmydesktop --fps=30 --windowid=${GAZEBO_WINDOW_ID} --no-sound -o $OUTPUT \
  > $OUTPUT.record_output.txt 2>&1 &
echo -e "${GREEN}OK${NOCOLOR}"

# Wait until the playback ends.
wait_until_playback_ends

# Terminate Gazebo.
echo -n "Encoding video and storing in $OUTPUT ..."
killall -w gzserver gzclient recordmydesktop
echo -e "${GREEN}OK${NOCOLOR}"
