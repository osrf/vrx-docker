#!/usr/bin/env bash

# set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

team_config_DIR=${DIR}/../team_config/

LIST_OF_TEAMS="$(ls ${team_config_DIR})"
date_time=$(date +%Y-%m-%d.%H-%M-%S)
CONSOLE_OUTPUT_DIR=${DIR}/prepare_output/${date_time}

# Show teams that were found
echo -e "Found teams: \n$LIST_OF_TEAMS"
echo -e "Storing command outputs to $CONSOLE_OUTPUT_DIR\n"

for TEAM_NAME in ${LIST_OF_TEAMS}; do
  mkdir -p ${CONSOLE_OUTPUT_DIR}/${TEAM_NAME}
  ${DIR}/../prepare_team_wamv.bash "${TEAM_NAME}" > ${CONSOLE_OUTPUT_DIR}/${TEAM_NAME}/output.txt 2>&1
  # TODO: Check for errors in compliance?
done

echo -e "${GREEN}Finished preparing WAM-V URDFs for all teams${NOCOLOR}"
