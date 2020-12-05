#!/bin/bash

# generate_trial_video.bash: A bash script to generate a video from a Gazebo playback log file.
# Input: Gazebo log file state.log
# Output: Recorded video in .ogv
#
# E.g.: ./generate_trial_video.bash -n example_team station_keeping 0
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
  echo "Usage: $0 [-n --nvidia] <team_name> <task_name> <trial_num>"
  exit 1
}

# Parse arguments
nvidia_arg=""
image_nvidia=""

POSITIONAL=()
while [[ $# -gt 0 ]]
do
  key="$1"

  case $key in
      -n|--nvidia)
      nvidia_arg="-n"
      image_nvidia="-nvidia"
      shift
      ;;
      *)    # unknown option
      POSITIONAL+=("$1")
      shift
      ;;
  esac
done

set -- "${POSITIONAL[@]}"

# Call usage() function if arguments not supplied.
[[ $# -ne 3 ]] && usage

TEAM_NAME=$1
TASK_NAME=$2
TRIAL_NUM=$3

# Constants for containers
SERVER_CONTAINER_NAME=vorc-server-system
ROS_DISTRO=melodic
LOG_DIR=/vorc/logs
NETWORK=vorc-network
NETWORK_SUBNET="172.16.0.10/16" # subnet mask allows communication between IP addresses with 172.16.xx.xx (xx = any)
SERVER_ROS_IP="172.16.0.22"
COMPETITOR_ROS_IP="172.16.0.20"
ROS_MASTER_URI="http://${SERVER_ROS_IP}:11311"

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Create the network for the containers to talk to each other.
${DIR}/utils/vorc_network.bash "${NETWORK}" "${NETWORK_SUBNET}"

echo "Playing back $TEAM_NAME solution in $TASK_NAME $TRIAL_NUM"
echo -e "=================================\n"

echo "Setting up"
echo "---------------------------------"

# Input file
HOST_LOG_DIR=${DIR}/generated/logs/${TEAM_NAME}/${TASK_NAME}/${TRIAL_NUM}
LOG_SUFFIX=/gazebo-server/state.log
LOG_FILE=${LOG_DIR}${LOG_SUFFIX}
HOST_LOG_FILE=${HOST_LOG_DIR}${LOG_SUFFIX}
# Sanity check: Make sure that the log file exists.
if [ ! -f $HOST_LOG_FILE ]; then
  echo "Gazebo log file [$HOST_LOG_FILE] not found!"
  exit 1
else
  echo "Found Gazebo log file [$HOST_LOG_FILE]"
fi

# Output directory
HOST_OUTPUT_DIR=${HOST_LOG_DIR}/video
OUTPUT_DIR=${LOG_DIR}/video
if [ -d "$HOST_OUTPUT_DIR" ]; then
  echo "Overwriting directory: ${HOST_OUTPUT_DIR}"
  rm -R $HOST_OUTPUT_DIR
else
  echo "Creating directory: ${HOST_OUTPUT_DIR}"
fi

mkdir -p $HOST_OUTPUT_DIR
OUTPUT_SUFFIX=/playback_video.ogv
HOST_OUTPUT=$HOST_OUTPUT_DIR$OUTPUT_SUFFIX
OUTPUT=$OUTPUT_DIR$OUTPUT_SUFFIX

# Ensure any previous containers are killed and removed.
${DIR}/utils/kill_vorc_containers.bash

echo "Starting simulation server container for playback"
echo "-------------------------------------------------"

# Define constants for recording
x=100
y=100
width=1000
height=750
BLACK_WINDOW_TIME=2

HOST_GZ_GUI_CONFIG_DIR=${DIR}/generated/logs/playback_gazebo
GZ_GUI_CONFIG_DIR=/home/$USER/.gazebo
if [ -d "$HOST_GZ_GUI_CONFIG_DIR" ]; then
  echo "Overwriting directory: ${HOST_GZ_GUI_CONFIG_DIR}"
  rm -R $HOST_GZ_GUI_CONFIG_DIR
else
  echo "Creating directory: ${HOST_GZ_GUI_CONFIG_DIR}"
fi
mkdir -p $HOST_GZ_GUI_CONFIG_DIR

# Tell gazebo client what size and place it should be
echo "[geometry]
width=$width
height=$height
x=$x
y=$y" > ${HOST_GZ_GUI_CONFIG_DIR}/gui.ini

# Run Gazebo simulation server container
SERVER_CMD="/play_vorc_log.sh ${LOG_FILE} ${OUTPUT}"
SERVER_IMG="vorc-server-${ROS_DISTRO}${image_nvidia}:latest"
${DIR}/vorc_server/run_container.bash $nvidia_arg ${SERVER_CONTAINER_NAME} $SERVER_IMG \
  "--net ${NETWORK} \
  --ip ${SERVER_ROS_IP} \
  -v ${HOST_LOG_DIR}:${LOG_DIR} \
  -v ${HOST_OUTPUT_DIR}:${OUTPUT_DIR} \
  -v ${HOST_GZ_GUI_CONFIG_DIR}:${GZ_GUI_CONFIG_DIR} \
  -e ROS_MASTER_URI=${ROS_MASTER_URI} \
  -e ROS_IP=${SERVER_ROS_IP} \
  -e VRX_DEBUG=false" \
  "${SERVER_CMD}" &
SERVER_PID=$!

echo "Waiting for server to start up"
sleep 9s

echo "Setting up screen recording..."

ADVERTISE_CMD="bash /home/master/vorc_ws/install/setup.bash && rostopic pub /record_ready std_msgs/Bool 0"
docker exec -it ${SERVER_CONTAINER_NAME} ${ADVERTISE_CMD}

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
  exit 1
fi

# Move Gazebo window to front
wmctrl -i -a ${GAZEBO_WINDOW_ID}

# Adjust the value of this constant if needed to avoid capturing a black
# screen for a long time.
sleep $BLACK_WINDOW_TIME

# Unpause the simulation right before starting recordmydesktop
echo "Injecting signal to unpause Gazebo..."
# Inject command into Docker container
UNPAUSE_CMD="bash /home/master/vorc_ws/install/setup.bash && rostopic pub /record_ready std_msgs/Bool 1"
docker exec -it ${SERVER_CONTAINER_NAME} ${UNPAUSE_CMD}
echo -e "${GREEN}OK${NOCOLOR}"

# Start recording the Gazebo Window.
echo "Recording screen..."
recordmydesktop --fps=30 --windowid=${GAZEBO_WINDOW_ID} --no-sound -o $HOST_OUTPUT \
  > $HOST_OUTPUT.record_output.txt 2>&1 #&
echo -e "${GREEN}OK${NOCOLOR}"

echo "Encoding video and storing in $OUTPUT ..."
killall -w recordmydesktop
echo -e "${GREEN}OK${NOCOLOR}"

# Wait for Docker container to terminate
wait $SERVER_PID

echo "Playback ended"
echo "---------------------------------"

# Kill and remove all containers before exit
${DIR}/utils/kill_vorc_containers.bash

exit 0
