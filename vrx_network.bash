#!/usr/bin/env bash

# Check if vrx-network already running
inspect=`docker network inspect vrx-network`
if [ inspect ]
then
  echo "replacing vrx-network"
  docker network rm vrx-network
fi

docker network create vrx-network
