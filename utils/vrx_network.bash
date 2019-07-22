#!/usr/bin/env bash

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

NETWORK=$1
SUBNET=$2

# Check if ${NETWORK} already running
if [ ! "$(docker network ls | grep ${NETWORK})" ]; then
  echo "Starting ${NETWORK}"
else
  echo "Replacing ${NETWORK}"
  docker network rm ${NETWORK}
fi

# Create network with subnet
docker network create --subnet "${SUBNET}" ${NETWORK}

echo -e "${GREEN}Done.${NOCOLOR}\n"
