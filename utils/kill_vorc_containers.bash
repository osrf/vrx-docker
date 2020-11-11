#!/usr/bin/env bash

# kill_vorc_containers.bash: A bash script that kills all vorc containers
# 
# eg. ./kill_vorc_containers.bash

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

kill_matching_containers () {
  echo "Killing any running Docker containers matching '$1'..."
  docker ps -a | grep "$1" | awk '{print $1}' | xargs --no-run-if-empty docker kill
  sleep 1

  # It's possible that the container is not running, but still exists.
  echo "Removing any Docker containers matching '$1'..."
  docker ps -a | grep "$1" | awk '{print $1}' | xargs --no-run-if-empty docker rm
  sleep 1
}

echo "Killing containers"
# TODO(mabelzhang): Change to vorc-competitor when have competitor dockerfile
kill_matching_containers "vrx-competitor-*"
kill_matching_containers "vorc-server-*"
echo -e "${GREEN}Done.${NOCOLOR}\n"
