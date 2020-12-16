#!/bin/bash

# prepare_task_trials.bash: A bash script to generate single tasks's world files, one for each task.
#
# E.g.: ./prepare_task_trials.bash stationkeeping

set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <task>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 1 ]] && usage

TASK_NAME=$1

echo -e "${GREEN}Preparing all trial worlds for task: ${TASK_NAME}${NOCOLOR}"

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Find task yaml
echo "Looking for config files"
TASK_CONFIG_DIR=${DIR}/task_config
TASK_CONFIG=${TASK_CONFIG_DIR}/${TASK_NAME}.yaml

if [ -f "${TASK_CONFIG}" ]; then
  echo -e "Successfully found: ${TASK_CONFIG}\n"
else
  echo -e "${RED}Err: ${TASK_CONFIG} not found."; exit 1;
fi

# Create directories to store generated files
world_xacro_target=${DIR}/generated/task_generated/${TASK_NAME}/world_xacros/
world_target=${DIR}/generated/task_generated/${TASK_NAME}/worlds/
mkdir -p ${world_xacro_target}
mkdir -p ${world_target}

extra_gen_args=""
if [ "${TASK_NAME}" = "gymkhana" ]; then
  config_target=${DIR}/generated/task_generated/${TASK_NAME}/config/
  mkdir -p ${config_target}
  extra_gen_args="config_target:=${config_target}"
fi

# Generate worlds
echo "Generating worlds..."
roslaunch vrx_gazebo generate_worlds.launch requested:=$TASK_CONFIG world_xacro_target:=$world_xacro_target world_target:=$world_target competition_pkg:="vorc_gazebo" world_name:="vorc_example_course" ${extra_gen_args}
echo -e "${GREEN}OK${NOCOLOR}\n"
echo -e "${GREEN}If successful, worlds are generated in \n  $world_xacro_target and \n  $world_target${NOCOLOR}\n"
