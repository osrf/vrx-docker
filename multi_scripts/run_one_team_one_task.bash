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
  echo "Usage: $0 [-n --nvidia] <team_name> <task_name>"
  exit 1
}

# Parse arguments
nvidia_arg=""

POSITIONAL=()
while [[ $# -gt 0 ]]
do
  key="$1"

  case $key in
      -n|--nvidia)
      nvidia_arg="-n"
      shift
      ;;
      *)    # unknown option
      POSITIONAL+=("$1")
      shift
      ;;
  esac
done

set -- "${POSITIONAL[@]}"

# Call usage() function if arguments not supplied.
[[ $# -ne 2 ]] && usage

TEAM_NAME=$1
TASK_NAME=$2

trial_DIR=${DIR}/../generated/task_generated/${TASK_NAME}/worlds/

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

successful_task=true

for TRIAL in ${LIST_OF_TRIALS}; do
  TRIAL_NUM=${TRIAL: -1}
  echo "Running ${TASK_NAME} trial number ${TRIAL_NUM}..."

  # Prepare directory for console output
  CONSOLE_OUTPUT_DIR=${DIR}/../generated/multi_scripts/run_output/${TEAM_NAME}/${TASK_NAME}/${TRIAL_NUM}
  mkdir -p ${CONSOLE_OUTPUT_DIR}

  # Run trial and store console output
  ${DIR}/../run_trial.bash $nvidia_arg "${TEAM_NAME}" "${TASK_NAME}" "${TRIAL_NUM}" > ${CONSOLE_OUTPUT_DIR}/output.txt 2>&1
  exit_status=$?

  # Print OK or FAIL message
  if [ $exit_status -eq 0 ]; then
    echo -e "${GREEN}OK.${NOCOLOR}"
  else
    echo -e "${RED}TRIAL FAILED: ${TEAM_NAME}/${TASK_NAME}/${TRIAL_NUM}${NOCOLOR}" >&2
    successful_task=false
  fi
done

# Record task score if all trials successful
if [ "$successful_task" = true ]; then
  echo "All $TASK_NAME trials completed. Creating text file for task score"
  python3 ${DIR}/../utils/get_task_score.py $TEAM_NAME $TASK_NAME
  exit_status=$?
  
  # Print OK or FAIL message
  if [ $exit_status -eq 0 ]; then
    echo -e "${GREEN}OK.${NOCOLOR}"
    exit 0
  else
    echo -e "${RED}TASK SCORE TEXT FILE CREATION FAILED: ${TEAM_NAME}/${TASK_NAME}${NOCOLOR}" >&2
    exit 1
  fi
else
  echo -e "${RED}All $TASK_NAME trials completed. >=1 trial was unsuccessful, so not creating text file for task score${NOCOLOR}" >&2
  exit 1
fi

