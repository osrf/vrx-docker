#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" 

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

TEAM_NAME=$1

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

echo -e "
${GREEN}Running all tasks for team: ${TEAM_NAME}${NOCOLOR}"

LIST_OF_TASKS="$(get_list_of_tasks)"

for TASK_NAME in ${LIST_OF_TASKS}; do
  echo "Running task: ${TASK_NAME}..."
  ${DIR}/run_one_team_one_task.bash "${TEAM_NAME}" "${TASK_NAME}"
done
