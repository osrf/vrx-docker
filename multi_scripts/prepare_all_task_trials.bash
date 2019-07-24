#!/usr/bin/env bash

# prepare_all_task_trials.bash: A bash script to run prepare_task_trials.bash on all yaml files in task_generated
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

task_generated_DIR=${DIR}/../task_generated/

# Get the available tasks from the generated directory
LIST_OF_TASKS="$(ls ${task_generated_DIR})"
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
