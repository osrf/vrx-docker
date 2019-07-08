#!/bin/bash

# prepare_team_wamv.bash: A shell script to generate single team's WAM-V urdf file.
# E.g.: ./prepare_team_wamv.bash example_team

set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <team>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 1 ]] && usage

TEAM_NAME=$1

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Find sensor and thruster yaml files
echo "Looking for config files"
TEAM_CONFIG_DIR=${DIR}/team_config/${TEAM_NAME}
SENSOR_CONFIG=${TEAM_CONFIG_DIR}/sensor_config.yaml
THRUSTER_CONFIG=${TEAM_CONFIG_DIR}/thruster_config.yaml

if [ -f "${SENSOR_CONFIG}" ]; then
  echo "Successfully found: ${SENSOR_CONFIG}"
else
  echo -e "${RED}Err: ${SENSOR_CONFIG}"; exit 1;
fi
if [ -f "${THRUSTER_CONFIG}" ]; then
  echo "Successfully found: ${THRUSTER_CONFIG}"
else
  echo -e "${RED}Err: ${THRUSTER_CONFIG}"; exit 1;
fi

# Create directory for generated files
wamv_target_dir=${DIR}/team_generated/${TEAM_NAME}
wamv_target=${wamv_target_dir}/${TEAM_NAME}.urdf
mkdir -p ${wamv_target_dir}

# Generate WAM-V
echo "Generating WAM-V..."
roslaunch vrx_gazebo generate_wamv.launch sensor_yaml:=$SENSOR_CONFIG thruster_yaml:=$THRUSTER_CONFIG wamv_target:=$wamv_target &

# Wait until generation is complete, then kill process
generate_wamv_pid=$!
sleep 3s
echo -e "${GREEN}OK${NOCOLOR}\n"
echo "Killing ${generate_wamv_pid}"
kill ${generate_wamv_pid}

# Move generated files to correct location
mv ${TEAM_CONFIG_DIR}/sensor_config.xacro ${wamv_target_dir}
mv ${TEAM_CONFIG_DIR}/thruster_config.xacro ${wamv_target_dir}
