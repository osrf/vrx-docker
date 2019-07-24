#!/usr/bin/env bash

# vrx_network.bash: A bash script that creates a Docker network
#
# eg. ./vrx_network.bash vrx-network 172.16.0.10/16

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <network_name> <subnet>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 2 ]] && usage

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
