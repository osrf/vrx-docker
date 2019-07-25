#!/usr/bin/env bash

# run_trial.bash: A bash script to run a trial for a given team. 
#
# E.g.: ./run_trial.bash example_team station_keeping 0
#
# Note: must run prepare scripts for the trial and team before running
#

# Comment out to allow script to continue, even if it encounters an error
set -e 

TEAM_NAME=$1
TASK_NAME=$2
TRIAL_NUM=$3

# Constants.
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

# Call usage() function if arguments not supplied.
[[ $# -ne 3 ]] && usage

# Constants for containers
SERVER_CONTAINER_NAME=vrx-server-system
ROS_DISTRO=melodic
LOG_DIR=/vrx/logs
NETWORK=vrx-network
NETWORK_SUBNET="172.16.0.10/16" # subnet mask allows communication between IP addresses with 172.16.xx.xx (xx = any)
SERVER_ROS_IP="172.16.0.22"
COMPETITOR_ROS_IP="172.16.0.20"
ROS_MASTER_URI="http://${SERVER_ROS_IP}:11311"

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Running $TEAM_NAME WAM-V in $TASK_NAME $TRIAL_NUM"
echo -e "=================================\n"

echo "Setting up"
echo "---------------------------------"

# Create the directory that logs will be copied into. Since the userid of the user in the container
# might different to the userid of the user running this script, we change it to be public-writable.
HOST_LOG_DIR=${DIR}/logs/${TEAM_NAME}/${TASK_NAME}/${TRIAL_NUM}
if [ -d "$HOST_LOG_DIR" ]; then
  echo "Overwriting directory: ${HOST_LOG_DIR}"
  rm -R $HOST_LOG_DIR
else
  echo "Creating directory: ${HOST_LOG_DIR}"
fi

mkdir -p ${HOST_LOG_DIR}
chmod 777 ${HOST_LOG_DIR}
echo -e "${GREEN}Done.${NOCOLOR}\n"

# Find wamv urdf and task world files
echo "Looking for generated files"
TEAM_GENERATED_DIR=${DIR}/team_generated/${TEAM_NAME}
if [ -f "${TEAM_GENERATED_DIR}/${TEAM_NAME}.urdf" ]; then
  echo "Successfully found: ${TEAM_GENERATED_DIR}/${TEAM_NAME}.urdf"
else
  echo -e "${RED}Err: ${TEAM_GENERATED_DIR}/${TEAM_NAME}.urdf not found."; exit 1;
fi

COMP_GENERATED_DIR=${DIR}/task_generated/${TASK_NAME}
if [ -f "${COMP_GENERATED_DIR}/worlds/${TASK_NAME}${TRIAL_NUM}.world" ]; then
  echo "Successfully found: ${COMP_GENERATED_DIR}/worlds/${TASK_NAME}${TRIAL_NUM}.world"
else
  echo -e "${RED}Err: ${COMP_GENERATED_DIR}/worlds/${TASK_NAME}${TRIAL_NUM}.world not found."; exit 1;
fi
echo -e "${GREEN}Done.${NOCOLOR}\n"

# Ensure any previous containers are killed and removed.
${DIR}/utils/kill_vrx_containers.bash

# Create the network for the containers to talk to each other.
${DIR}/utils/vrx_network.bash "${NETWORK}" "${NETWORK_SUBNET}"

echo "Starting simulation server container"
echo "---------------------------------"

# TODO: Figure out if we can start competitor container first, so simulation doesn't start too early, but may have issues if
#       competitior container waiting for ROS master and has error before server is created.
# Run Gazebo simulation server container
SERVER_CMD="/run_vrx_trial.sh /team_generated/${TEAM_NAME}.urdf /task_generated/worlds/${TASK_NAME}${TRIAL_NUM}.world ${LOG_DIR}"
${DIR}/vrx_server/run_container.bash ${SERVER_CONTAINER_NAME} vrx-server-${ROS_DISTRO}:latest \
  "--net ${NETWORK} \
  --ip ${SERVER_ROS_IP} \
  -v ${TEAM_GENERATED_DIR}:/team_generated \
  -v ${COMP_GENERATED_DIR}:/task_generated \
  -v ${HOST_LOG_DIR}:${LOG_DIR} \
  -e ROS_MASTER_URI=${ROS_MASTER_URI} \
  -e ROS_IP=${SERVER_ROS_IP} \
  -e VRX_EXIT_ON_COMPLETION=true \
  -e VRX_DEBUG=false" \
  "${SERVER_CMD}" &
SERVER_PID=$!

# Wait until server starts before competitor code can be run
echo "Waiting for server to start up"
sleep 9s

echo "Starting competitor container"
echo "---------------------------------"

echo -e "\nLooking for dockerhub_image.txt"
DOCKERHUB_IMAGE_FILENAME=${DIR}/team_config/${TEAM_NAME}/dockerhub_image.txt
if [ -f "${DOCKERHUB_IMAGE_FILENAME}" ]; then
  echo "Successfully found: ${DOCKERHUB_IMAGE_FILENAME}"
  DOCKERHUB_IMAGE=$(head -n 1 ${DOCKERHUB_IMAGE_FILENAME})
  echo -e "Dockerhub image: ${DOCKERHUB_IMAGE}\n"
else
  echo -e "${RED}Err: ${DOCKERHUB_IMAGE_FILENAME} not found."; exit 1;
fi

# Start the competitors container and let it run in the background.
docker login
echo -e "Creating container for ${DOCKERHUB_IMAGE}\n"
COMPETITOR_CONTAINER_NAME="vrx-competitor-system"
docker run \
    --net ${NETWORK} \
    --name $COMPETITOR_CONTAINER_NAME \
    --env ROS_MASTER_URI=${ROS_MASTER_URI} \
    --env ROS_IP=${COMPETITOR_ROS_IP} \
    --ip ${COMPETITOR_ROS_IP} \
    ${DOCKERHUB_IMAGE} &

# Run competition until server is ended
wait $SERVER_PID
echo "Trial ended. Logging data"
echo "---------------------------------"

# Copy the ROS log files from the competitor's container.
# Copy the ROS log files from the server's container.
echo "Copying ROS log files from server container..."
docker cp --follow-link ${SERVER_CONTAINER_NAME}:/home/$USER/.ros/log/latest $HOST_LOG_DIR/ros-server-latest
docker cp --follow-link ${SERVER_CONTAINER_NAME}:/home/$USER/.gazebo/ $HOST_LOG_DIR/gazebo-server
docker cp --follow-link ${SERVER_CONTAINER_NAME}:/home/$USER/vrx_rostopics.bag $HOST_LOG_DIR/
docker cp --follow-link ${SERVER_CONTAINER_NAME}:/home/$USER/verbose_output.txt $HOST_LOG_DIR/

echo -e "${GREEN}OK${NOCOLOR}\n"

# Record trial score
echo "Creating text file for trial score"
python ${DIR}/utils/get_trial_score.py $TEAM_NAME $TASK_NAME $TRIAL_NUM
echo -e "${GREEN}OK${NOCOLOR}\n"

# TODO: find way to record log files, even if competitor's container's log files not in /root/.ros/log
echo "Copying ROS log files from competitor container..."
docker cp --follow-link $COMPETITOR_CONTAINER_NAME:/root/.ros/log $HOST_LOG_DIR/ros-competitor
echo -e "${GREEN}OK${NOCOLOR}\n"

# Kill and remove all containers before exit
${DIR}/utils/kill_vrx_containers.bash

exit 0
