#!/usr/bin/env bash

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

NETWORK=$1

# Check if ${NETWORK} already running
inspect=`docker network inspect ${NETWORK}`
if [ inspect ]; then
  echo "Replacing ${NETWORK}"
  docker network rm ${NETWORK}
else
  echo "Starting ${NETWORK}"
fi

docker network create ${NETWORK}
echo -e "${GREEN}Done.${NOCOLOR}\n"
