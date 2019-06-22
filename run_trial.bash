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
  echo -e "${RED}Err: ${TEAM_CONFIG_DIR}/team_config.yaml not found."; exit 1;
fi

COMP_CONFIG_DIR=${DIR}/trial_config
if [ -f "${COMP_CONFIG_DIR}/${TRIAL_NAME}.yaml" ]; then
  echo "Successfully found: ${COMP_CONFIG_DIR}/${TRIAL_NAME}.yaml"
else
  echo -e "${RED}Err: ${COMP_CONFIG_DIR}/${TRIAL_NAME}.yaml not found."; exit 1;
fi

# Ensure any previous containers are killed and removed.
${DIR}/kill_vrx_containers.bash

# Create the network for the containers to talk to each other.
${DIR}/vrx_network.bash

# Start the competition server. When the trial ends, the container will be killed.
# The trial may end because of time-out, because of completion, or because the user called the
# /vrx/end_competition service.
${DIR}/vrx_server/run_container.bash ${SERVER_CONTAINER_NAME} vrx-server-${ROS_DISTRO}:latest \
  "-v ${TEAM_CONFIG_DIR}:/team_config \
  -v ${COMP_CONFIG_DIR}:/trial_config \
  -v ${HOST_LOG_DIR}:${LOG_DIR} \
  -e vrx_EXIT_ON_COMPLETION=1" \
  "/run_vrx_task.sh /trial_config/${TRIAL_NAME}.yaml /team_config/team_config.yaml ${LOG_DIR}" &

echo "Waiting for server to start up"
sleep 20s
# Start the competitors container and let it run in the background.
# COMPETITOR_IMAGE_NAME="vrx_competitor_${TEAM_NAME}"
COMPETITOR_IMAGE_NAME="ros:ros-tutorials"
RATE=1
CMD=2
# COMPETITOR_RUN_SYSTEM_CMD="echo $ROS_IP && rostopic pub /left_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD}"
COMPETITOR_RUN_SYSTEM_CMD="rostopic pub /left_thrust_cmd std_msgs/Float32 -r 1 -- 2"
#COMPETITOR_RUN_SYSTEM_CMD="\"rostopic pub /left_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD} & rostopic pub /right_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD}\""
echo "Starting competitor command"
docker run --rm \
    --net vrx-network \
    --name vrx-competitor-test-1 \
    --env ROS_HOSTNAME=vrx-competitor-test-1 \
    --env ROS_MASTER_URI=http://vrx_server_system:11311 \
    --env ROS_IP=172.19.0.3 \
    ${COMPETITOR_IMAGE_NAME} \
    ${COMPETITOR_RUN_SYSTEM_CMD} &

COMPETITOR_IMAGE_NAME="ros:ros-tutorials"
RATE=1
CMD=2
# COMPETITOR_RUN_SYSTEM_CMD="echo $ROS_IP && rostopic pub /left_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD}"
COMPETITOR_RUN_SYSTEM_CMD="rostopic echo /left_thrust_cmd"
#COMPETITOR_RUN_SYSTEM_CMD="\"rostopic pub /left_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD} & rostopic pub /right_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD}\""
echo "Starting competitor command"
docker run --rm \
    --net vrx-network \
    --name vrx-competitor-test-2 \
    --env ROS_HOSTNAME=vrx-competitor-test-2 \
    --env ROS_MASTER_URI=http://vrx_server_system:11311 \
    --env ROS_IP=172.19.0.4 \
    ${COMPETITOR_IMAGE_NAME} \
    ${COMPETITOR_RUN_SYSTEM_CMD}
echo "Passed competitor command"

echo "Run trial sleep for 30s"
sleep 30s
# Copy the ROS log files from the competitor's container.
# echo "Copying ROS log files from competitor container..."
# docker cp --follow-link ${COMPETITOR_IMAGE_NAME}-system:/root/.ros/log/latest $HOST_LOG_DIR/ros-competitor
# echo -e "${GREEN}OK${NOCOLOR}"

# Kill and remove all containers before exit
./kill_vrx_containers.bash

exit 0
