#!/usr/bin/env bash

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Check if vrx-network already running
inspect=`docker network inspect vrx-network`
if [ inspect ]; then
  echo "Replacing vrx-network"
  docker network rm vrx-network
else
  echo "Starting vrx-network"
fi

docker network create vrx-network
echo -e "${GREEN}Done.${NOCOLOR}\n"
