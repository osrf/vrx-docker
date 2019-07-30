#!/usr/bin/env bash

# run_all_teams_all_tasks.bash: A bash script to run run_one_team_all_tasks.bash on all teams in team_generated
#
# eg. ./run_all_teams_all_tasks.bash

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

team_generated_DIR=${DIR}/../generated/team_generated/

LIST_OF_TEAMS="$(ls ${team_generated_DIR})"

for TEAM_NAME in ${LIST_OF_TEAMS}; do
  ${DIR}/run_one_team_all_tasks.bash "${TEAM_NAME}"
done

echo -e "${GREEN}Finished running all teams for all tasks${NOCOLOR}"
