#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

TEAM_NAME=$1
TASK_NAME=$2

trial_DIR=${DIR}/../task_generated/${TASK_NAME}/worlds/

# Get the available trial numbers from the trial directory
get_list_of_trial_nums()
{
  world_files=$(ls ${trial_DIR}/*.world)

  for f in $(ls ${trial_DIR}/*.world); do
    f=${f##*/}
    f=${f//.world}
    f=${f//world}
    all_trial_nums="${all_trial_nums} ${f}"
  done

  echo $all_trial_nums
}

echo -e "
${GREEN}Running all ${TASK_NAME} trials for team: ${TEAM_NAME}${NOCOLOR}"

LIST_OF_TRIAL_NUMS="$(get_list_of_trial_nums)"

for TRIAL_NUM in ${LIST_OF_TRIAL_NUMS}; do
  echo "Running ${TASK_NAME} trial number ${TRIAL_NUM}..."
  CONSOLE_OUTPUT_DIR=logs/$(date +%Y-%m-%d.%H-%M-%S)_multi_scripts_output/${TEAM_NAME}/${TASK_NAME}/${TRIAL_NUM}
  mkdir -p ${CONSOLE_OUTPUT_DIR}
  ./run_trial.bash "${TEAM_NAME}" "${TASK_NAME}" "${TRIAL_NUM}" > ${CONSOLE_OUTPUT_DIR}/output.txt 2>&1
  exit_status=$?
  if [ $exit_status -eq 0 ]; then
    echo -e "${GREEN}OK.${NOCOLOR}"
  else
    echo -e "${RED}TRIAL FAILED: ${TEAM_NAME}/${TASK_NAME}/${TRIAL_NUM}${NOCOLOR}" >&2
  fi
done
