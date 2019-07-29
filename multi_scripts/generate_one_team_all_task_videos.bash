#!/usr/bin/env bash

# generate_one_team_all_task_videos.bash: A bash script that calls generate_one_team_one_task_videos.bash on all tasks in logs/<team>
#
# eg. ./generate_one_team_all_task_videos.bash example_team

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

task_DIR=${DIR}/../logs/${TEAM_NAME}

echo -e "
Generating videos for all tasks for team: ${TEAM_NAME}"
echo "========================================================="

LIST_OF_TASKS="$(ls $task_DIR)"

successful_team=true

for TASK_NAME in ${LIST_OF_TASKS}; do
  # Check that it is a directory
  if [ -d "$task_DIR/$TASK_NAME" ]; then
    echo "Generating video for task: ${TASK_NAME}..."
    echo "-----------------------------------"
    ${DIR}/generate_one_team_one_task_videos.bash "${TEAM_NAME}" "${TASK_NAME}"

    # Check if successful
    if [ $? -ne 0 ]; then
      successful_team=false
    fi
  fi
done



# Output success or failure
if [ "$successful_team" = true ]; then
  echo -e "${GREEN}Successfully generated all trial videos for all tasks for team $TEAM_NAME.${NOCOLOR}"
else
  echo -e "${RED}All ${TEAM_NAME} $TASK_NAME trial videos completed for all tasks. >=1 task was unsuccessful.${NOCOLOR}" >&2
  exit 1
fi
