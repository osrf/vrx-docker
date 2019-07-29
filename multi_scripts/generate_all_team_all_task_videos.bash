#!/usr/bin/env bash

# generate_all_team_all_task_videos.bash: A bash script to run generate_one_team_all_task_videos.bash on all teams in logs
#
# eg. ./generate_all_team_all_task_videos.bash

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

team_DIR=${DIR}/../logs

LIST_OF_TEAMS="$(ls ${team_DIR})"

for TEAM_NAME in ${LIST_OF_TEAMS}; do
  ${DIR}/generate_one_team_all_task_videos.bash "${TEAM_NAME}"
done

echo -e "${GREEN}Finished generating all videos for all teams for all tasks${NOCOLOR}"
