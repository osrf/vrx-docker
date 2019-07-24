#!/usr/bin/env bash

# run_one_team_all_tasks.bash: A bash script that calls run_one_team_one_task.bash on all tasks in task_config
#
# eg. ./run_one_team_all_tasks.bash example_team

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" 

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <team_name>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 1 ]] && usage

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
echo "========================================================="

LIST_OF_TASKS="$(get_list_of_tasks)"

for TASK_NAME in ${LIST_OF_TASKS}; do
  echo "Running task: ${TASK_NAME}..."
  echo "-----------------------------------"
  ${DIR}/run_one_team_one_task.bash "${TEAM_NAME}" "${TASK_NAME}"
done

# Record team score
echo "${TEAM_NAME} has completed all tasks. Creating text file for team score"
python ${DIR}/../utils/get_team_score.py $TEAM_NAME
echo -e "${GREEN}OK${NOCOLOR}\n"

