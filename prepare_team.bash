#!/bin/bash

# prepare_team.bash: A bash script to set up team file structure
#
# E.g.: ./prepare_team.bash example_team

set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <team>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 1 ]] && usage

TEAM_NAME=$1

echo -e "${GREEN}Preparing team: ${TEAM_NAME}${NOCOLOR}"

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Create directory for generated files
target_dir=${DIR}/generated/team_generated
mkdir -p ${target_dir}
target_file=${target_dir}/${TEAM_NAME}

touch ${target_file}
echo -e "${GREEN}OK${NOCOLOR}"
echo -e "${GREEN}File generated in ${target_file}${NOCOLOR}"
