#!/usr/bin/env bash

# prepare_all_task_trials.bash: A bash script to run prepare_task_trials.bash on all yaml files in task_config
#
# All terminal output it piped to multi_scripts/prepare_output to keep things clear
#
# eg. ./prepare_all_task_trials.bash


# set -e

echo "Preparing all task trials"
echo "================================="

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 0 ]] && usage

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

task_config_DIR=${DIR}/../task_config/

# Get the available tasks from the config directory
get_list_of_tasks()
{
  yaml_files=$(ls ${task_config_DIR}/*.yaml)

  for f in $(ls ${task_config_DIR}/*.yaml); do
    f=${f##*/}
    f=${f//.yaml}
    all_names="${all_names} ${f}"
  done

  echo $all_names
}

LIST_OF_TASKS="$(get_list_of_tasks)"
CONSOLE_OUTPUT_DIR=${DIR}/prepare_output

# Show tasks that were found
echo "Found tasks: $LIST_OF_TASKS"
echo -e "Storing command outputs to $CONSOLE_OUTPUT_DIR\n"

for TASK_NAME in ${LIST_OF_TASKS}; do
  echo -e "Preparing all trial worlds for task: ${TASK_NAME}"

  mkdir -p ${CONSOLE_OUTPUT_DIR}/${TASK_NAME}
  ${DIR}/../prepare_task_trials.bash "${TASK_NAME}" > ${CONSOLE_OUTPUT_DIR}/${TASK_NAME}/output.txt 2>&1

  echo -e "${GREEN}OK${NOCOLOR}\n"
done

echo -e "${GREEN}Finished preparing trial worlds for all tasks${NOCOLOR}"
