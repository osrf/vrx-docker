#!/usr/bin/env bash

set -e

TEAM_NAME=$1
TRIAL_NAME=$2

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <team_name> <trial_name>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 2 ]] && usage

SERVER_CONTAINER_NAME=vrx_server_system
ROS_DISTRO=melodic
LOG_DIR=/vrx/logs

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Create the directory that logs will be copied into. Since the userid of the user in the container
# might different to the userid of the user running this script, we change it to be public-writable.
HOST_LOG_DIR=${DIR}/logs/${TEAM_NAME}/${TRIAL_NAME}
echo "Creating directory: ${HOST_LOG_DIR}"
mkdir -p ${HOST_LOG_DIR}
chmod 777 ${HOST_LOG_DIR}

# Find team and competition yaml files
TEAM_CONFIG_DIR=${DIR}/team_config/${TEAM_NAME}
if [ -f "${TEAM_CONFIG_DIR}/team_config.yaml" ]; then
  echo "Successfully found: ${TEAM_CONFIG_DIR}/team_config.yaml"
else
  echo -e "${RED}Err: ${TEAM_CONFIG_DIR}/team_config.yaml not fou    nd."; exit 1;
fi

COMP_CONFIG_DIR=${DIR}/trial_config
if [ -f "${COMP_CONFIG_DIR}/${TRIAL_NAME}.yaml" ]; then
  echo "Successfully found: ${COMP_CONFIG_DIR}/${TRIAL_NAME}.yaml    "
else
  echo -e "${RED}Err: ${COMP_CONFIG_DIR}/${TRIAL_NAME}.yaml not f    ound."; exit 1;
fi

# Ensure any previous containers are killed and removed.
${DIR}/kill_vrx_containers.bash

# Create the network for the containers to talk to each other.
${DIR}/vrx_network.bash

# Start the competitors container and let it run in the background.
COMPETITOR_IMAGE_NAME="vrx_competitor_${TEAM_NAME}"
# ./vrx_competitor/run_competitor_container.bash ${COMPETITOR_IMAGE_NAME} "/run_team_system_with_delay.bash" &

# Start the competition server. When the trial ends, the container will be killed.
# The trial may end because of time-out, because of completion, or because the user called the
# /vrx/end_competition service.
${DIR}/vrx_server/run_container.bash ${SERVER_CONTAINER_NAME} vrx-server-${ROS_DISTRO}:latest \
  "-v ${TEAM_CONFIG_DIR}:/team_config \
  -v ${COMP_CONFIG_DIR}:/trial_config \
  -v ${HOST_LOG_DIR}:${LOG_DIR} \
  -e vrx_EXIT_ON_COMPLETION=1" \
  "/run_vrx_task.sh /trial_config/${TRIAL_NAME}.yaml /team_config/team_config.yaml ${LOG_DIR}"

# Copy the ROS log files from the competitor's container.
# echo "Copying ROS log files from competitor container..."
# docker cp --follow-link ${COMPETITOR_IMAGE_NAME}-system:/root/.ros/log/latest $HOST_LOG_DIR/ros-competitor
# echo -e "${GREEN}OK${NOCOLOR}"

# Kill and remove all containers before exit
./kill_vrx_containers.bash

exit 0
