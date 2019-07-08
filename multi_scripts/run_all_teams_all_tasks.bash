#!/usr/bin/env bash

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

team_config_DIR=${DIR}/../team_config/

LIST_OF_TEAMS="$(ls ${team_config_DIR})"

for TEAM_NAME in ${LIST_OF_TEAMS}; do
  ${DIR}/run_one_team_all_tasks.bash "${TEAM_NAME}"
done
