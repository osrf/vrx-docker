#!/bin/bash

# prepare_trial_world.bash: A shell script to generate single trial's world files.
# E.g.: ./prepare_trial_world.bash example_trial

set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <trial>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 1 ]] && usage

TRIAL_NAME=$1

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Find trial yaml
echo "Looking for config files"
TRIAL_CONFIG_DIR=${DIR}/trial_config
TRIAL_CONFIG=${TRIAL_CONFIG_DIR}/${TRIAL_NAME}.yaml

if [ -f "${TRIAL_CONFIG}" ]; then
  echo "Successfully found: ${TRIAL_CONFIG}"
else
  echo -e "${RED}Err: ${TRIAL_CONFIG}"; exit 1;
fi

# Create directories to store generated files
world_xacro_target=${DIR}/generated/trial/${TRIAL_NAME}/world_xacros/
world_target=${DIR}/generated/trial/${TRIAL_NAME}/worlds/
mkdir -p ${world_xacro_target}
mkdir -p ${world_target}

# Generate worlds
echo "Generating worlds..."
roslaunch vrx_gazebo generate_worlds.launch requested:=$TRIAL_CONFIG world_xacro_target:=$world_xacro_target world_target:=$world_target &
generate_worlds_pid=$!
sleep 5s
echo -e "${GREEN}OK${NOCOLOR}\n"

echo "Killing ${generate_worlds_pid}"
kill ${generate_worlds_pid}
