#!/usr/bin/env bash

# prepare_all_team_wamvs.bash: A bash script to run prepare_team_wamv.bash on all teams in team_config
#
# All terminal output it piped to multi_scripts/prepare_output to keep things clear
#
# eg. ./prepare_all_team_wamvs.bash

# set -e

echo "Preparing all team wamvs"
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

team_config_DIR=${DIR}/../team_config/

LIST_OF_TEAMS="$(ls ${team_config_DIR})"
CONSOLE_OUTPUT_DIR=${DIR}/prepare_output

# Show teams that were found
echo -e "Found teams: \n$LIST_OF_TEAMS"
echo -e "Storing command outputs to $CONSOLE_OUTPUT_DIR\n"

for TEAM_NAME in ${LIST_OF_TEAMS}; do
  echo -e "Preparing WAM-V URDF for team: ${TEAM_NAME}"

  mkdir -p ${CONSOLE_OUTPUT_DIR}/${TEAM_NAME}
  ${DIR}/../prepare_team_wamv.bash "${TEAM_NAME}" > ${CONSOLE_OUTPUT_DIR}/${TEAM_NAME}/output.txt 2>&1
  # TODO: Check for errors in compliance?

  echo -e "${GREEN}OK${NOCOLOR}\n"
done

echo -e "${GREEN}Finished preparing WAM-V URDFs for all teams${NOCOLOR}"
