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

LIST_OF_TRIALS="$(get_list_of_trial_nums)"
date_time=$(date +%Y-%m-%d.%H-%M-%S)

for TRIAL in ${LIST_OF_TRIALS}; do
  TRIAL_NUM=${TRIAL: -1}
  echo "Running ${TASK_NAME} trial number ${TRIAL_NUM}..."
  CONSOLE_OUTPUT_DIR=${DIR}/run_output/${date_time}/${TEAM_NAME}/${TASK_NAME}/${TRIAL_NUM}
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
