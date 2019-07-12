#!/usr/bin/env bash

# set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

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
date_time=$(date +%Y-%m-%d.%H-%M-%S)
CONSOLE_OUTPUT_DIR=${DIR}/prepare_output/${date_time}

# Show tasks that were found
echo "Found tasks: $LIST_OF_TASKS"
echo -e "Storing command outputs to $CONSOLE_OUTPUT_DIR\n"

for TASK_NAME in ${LIST_OF_TASKS}; do
  mkdir -p ${CONSOLE_OUTPUT_DIR}/${TASK_NAME}
  ${DIR}/../prepare_task_trials.bash "${TASK_NAME}" > ${CONSOLE_OUTPUT_DIR}/${TASK_NAME}/output.txt 2>&1
done

echo -e "${GREEN}Finished preparing trial worlds for all tasks${NOCOLOR}"
