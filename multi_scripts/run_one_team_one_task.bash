#!/usr/bin/env bash

# run_one_team_one_task.bash: A bash script that calls run_trial.bash on all trials in task_generated
#
# eg. ./run_one_team_one_task.bash example_team station_keeping

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <team_name> <task_name>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 2 ]] && usage

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

LIST_OF_TRIALS="$(get_list_of_trial_nums)"

for TRIAL in ${LIST_OF_TRIALS}; do
  TRIAL_NUM=${TRIAL: -1}
  echo "Running ${TASK_NAME} trial number ${TRIAL_NUM}..."
  CONSOLE_OUTPUT_DIR=${DIR}/run_output/${TEAM_NAME}/${TASK_NAME}/${TRIAL_NUM}
  mkdir -p ${CONSOLE_OUTPUT_DIR}
  ${DIR}/../run_trial.bash "${TEAM_NAME}" "${TASK_NAME}" "${TRIAL_NUM}" > ${CONSOLE_OUTPUT_DIR}/output.txt 2>&1
  exit_status=$?
  if [ $exit_status -eq 0 ]; then
    echo -e "${GREEN}OK.${NOCOLOR}"
  else
    echo -e "${RED}TRIAL FAILED: ${TEAM_NAME}/${TASK_NAME}/${TRIAL_NUM}${NOCOLOR}" >&2
  fi
done

# Record task score
echo "All $TASK_NAME trials completed. Creating text file for task score"
python ${DIR}/../utils/get_task_score.py $TEAM_NAME $TASK_NAME
echo -e "${GREEN}OK${NOCOLOR}\n"
