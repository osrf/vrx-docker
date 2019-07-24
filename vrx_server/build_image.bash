#!/bin/bash
echo "Building vrx-server image"
echo "================================="

set -x
set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Uncoment this line to rebuild without cache
#DOCKER_ARGS="--no-cache"

echo "Build image: vrx-server"

# Print commands to terminal
set -x

DOCKER_ARGS="--no-cache"
USERID=`id -u $USER`
if [[ ${USERID} != 0 ]]; then
  DOCKER_ARGS="--build-arg USERID=${USERID}"
fi

docker build --force-rm ${DOCKER_ARGS} --tag vrx-server-melodic:latest --build-arg USER=$USER --build-arg GROUP=$USER $DIR/vrx-server

set +x
echo -e "${GREEN}Done.${NOCOLOR}\n"
