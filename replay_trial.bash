#!/bin/bash

# replay_trial.bash: Replay a trial recorded to a Gazebo log file, e.g.
# state.log.
#
# Input: Gazebo log file state.log
#
# Usage:
# ./replay_trial.bash -n example_team station_keeping 0
#
# Dependencies:
# sudo apt-get install psmisc

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
  echo "Usage: $0 [-n --nvidia] [--keep-docker] [--manual-play] [--keep-gz] <team_name> <task_name> <trial_num>"
  echo "--keep-docker: Keep Gazebo window open and Docker container running after playback ends."
  echo "  By default, everything is terminated automatically."
  echo "--manual-play: Do not automatically start playback. Wait for user to click in GUI."
  echo "  By default, playack automatically starts."
  echo "--keep-gz: Keep Gazebo server and client running after playback ends."
  echo "  Implies --keep-docker. By default, they are shut down automatically."
  exit 1
}

# Parse arguments
nvidia_arg=""
image_nvidia=""
keep_docker=1

# Args to pass to script internal to Docker container
manual_play=""
keep_gz=""

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

    --keep-docker)
      keep_docker=1
      shift
      ;;

    --manual-play)
      manual_play="--manual-play"
      shift
      ;;

    --keep-gz)
      keep_docker=1
      keep_gz="--keep-gz"
      shift
      ;;

    # Treat unknown options as positional args
    *)
      POSITIONAL+=("$1")
      shift
      ;;
  esac
done

set -- "${POSITIONAL[@]}"

# Call usage() function if arguments not supplied.
[[ $# -ne 3 ]] && echo "Invalid arguments: $@" && usage

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
SERVER_CMD="/play_vorc_log.sh ${LOG_FILE} ${OUTPUT} ${manual_play} ${keep_gz}"
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

if [ $keep_docker -eq 0 ]; then
  # Wait for Docker container to terminate
  wait $SERVER_PID

  echo "Playback ended"
  echo "---------------------------------"

  # Kill and remove all containers before exit
  ${DIR}/utils/kill_vorc_containers.bash

  exit 0
fi
