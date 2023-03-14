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

# DOCKER_ARGS="$DOCKER_ARGS --no-cache"

# Parse arguments
BUILD_BASE=""
image_name="vrx-server-jammy"

set -- "${POSITIONAL[@]}"

# Usage
if [ $# -gt 0 ]
then
    echo "Usage: $0 "
    exit 1
fi

# Build image
echo "Build image: $image_name"
set -x
image_plus_tag=$image_name:$(export LC_ALL=C; date +%Y_%m_%d_%H%M)
docker build --force-rm ${DOCKER_ARGS} --tag $image_plus_tag $BUILD_BASE $DIR/vrx-server && \
docker tag $image_plus_tag $image_name:latest && \
echo "Built $image_plus_tag and tagged as $image_name:latest"

set +x
echo -e "${GREEN}Done.${NOCOLOR}\n"
