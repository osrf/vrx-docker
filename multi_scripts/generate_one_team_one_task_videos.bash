#!/usr/bin/env bash

# generate_one_team_one_task_videos.bash: A bash script that calls generate_trial_video.bash on all trials in logs/<team>/<task>
#
# eg. ./generate_one_team_one_task_videos.bash example_team station_keeping

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

set -x

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

trial_DIR=${DIR}/../logs/${TEAM_NAME}/${TASK_NAME}

# Get the available trial numbers from the trial directory

echo -e "
${GREEN}Generating videos for all ${TASK_NAME} trials for team: ${TEAM_NAME}${NOCOLOR}"

LIST_OF_TRIAL_NUMS="$(ls $trial_DIR)"

successful_task=true

re='^[0-9]+$'

for TRIAL_NUM in ${LIST_OF_TRIAL_NUMS}; do
  # Check if is a trial number
  if [[ $TRIAL_NUM =~ $re ]] ; then
    echo " ${TASK_NAME} trial number ${TRIAL_NUM}..."

    # Prepare directory for console output
    CONSOLE_OUTPUT_DIR=${DIR}/generate_video_output/${TEAM_NAME}/${TASK_NAME}/${TRIAL_NUM}
    mkdir -p ${CONSOLE_OUTPUT_DIR}

    # Generate trial video and store console output
    ${DIR}/../generate_trial_video.bash "${TEAM_NAME}" "${TASK_NAME}" "${TRIAL_NUM}" > ${CONSOLE_OUTPUT_DIR}/output.txt 2>&1
    exit_status=$?

    # Print OK or FAIL message
    if [ $exit_status -eq 0 ]; then
      echo -e "${GREEN}OK.${NOCOLOR}"
    else
      echo -e "${RED}TRIAL VIDEO FAILED: ${TEAM_NAME}/${TASK_NAME}/${TRIAL_NUM}${NOCOLOR}" >&2
      successful_task=false
    fi
  fi
done

# Output success or failure
if [ "$successful_task" = true ]; then
  echo -e "${GREEN}All $TASK_NAME trial videos completed successfully.${NOCOLOR}"
else
  echo -e "${RED}All $TASK_NAME trial videos completed. >=1 trial was unsuccessful.${NOCOLOR}" >&2
  exit 1
fi
