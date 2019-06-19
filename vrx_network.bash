#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# networking
inspect=`docker network inspect vrx-network`
if [ inspect ]
then
  echo "replacing vrx-network"
  docker network rm vrx-network
fi

docker network create -d bridge \
  --subnet=172.17.0.0/16 \
  vrx-network
