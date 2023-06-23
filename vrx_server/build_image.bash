#!/bin/bash

# build_image.bash: A bash script to build the vrx server image
#
# E.g.: ./build_image.bash    # non-Nvidia
#   or  ./build_image.bash -n # Nvidia

echo "Building vrx-server image"
echo "================================="

set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Parse arguments
local_base_name="vrx-local-base"
image_name="vrx-server-jammy"

DOCKER_ARGS="--build-arg BASEIMG=$local_base_name"
# DOCKER_ARGS="$DOCKER_ARGS --no-cache"
JOY=/dev/input/js0


set -- "${POSITIONAL[@]}"

# Usage
if [ $# -gt 0 ]
then
    echo "Usage: $0 "
    exit 1
fi

ROCKER_ARGS="--dev-helpers --devices $JOY --nvidia --x11 --user --user-override-name=developer  --image-name $local_base_name"
rocker ${ROCKER_ARGS} npslearninglab/watery_robots:vrx_base "echo -e '${GREEN}Created $local_base_name.${NOCOLOR}\n'"

CONTAINER_NAME="vrx-server-system"

# Build image
echo "Building image: <${image_name}> from <$local_base_name>"
set -x
image_plus_tag=${image_name}:$(export LC_ALL=C; date +%Y_%m_%d_%H%M)
docker build --force-rm ${DOCKER_ARGS} --tag $image_plus_tag $DIR/vrx-server && \
docker tag $image_plus_tag ${image_name}:latest && \
echo "Built $image_plus_tag and tagged as ${image_name}:latest"

#echo "Using image <$IMG_NAME> to start container <$CONTAINER_NAME>"
# --name $CONTAINER_NAME


set +x
