#!/bin/bash

# build_image.bash: A bash script to build the vorc server image
#
# E.g.: ./build_image.bash    # non-Nvidia
#   or  ./build_image.bash -n # Nvidia

echo "Building vorc-server image"
echo "================================="

set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Setup docker args
USERID=`id -u $USER`
if [[ ${USERID} != 0 ]]; then
  DOCKER_ARGS="--build-arg USERID=${USERID}"
fi

# DOCKER_ARGS="$DOCKER_ARGS --no-cache"

# Parse arguments
BUILD_BASE=""
image_name="vorc-server-melodic"

# Parse args related to NVIDIA
POSITIONAL=()
while [[ $# -gt 0 ]]
do
    key="$1"

    case $key in
        -n|--nvidia)
        BUILD_BASE="--build-arg BASEIMG=nvidia/opengl:1.0-glvnd-devel-ubuntu18.04"
        image_name="$image_name-nvidia"
        shift
        ;;
        *)    # unknown option
        POSITIONAL+=("$1")
        shift
        ;;
    esac
done

set -- "${POSITIONAL[@]}"

# Usage
if [ $# -gt 0 ]
then
    echo "Usage: $0 [-n --nvidia]"
    exit 1
fi

# Build image
echo "Build image: $image_name"
set -x
image_plus_tag=$image_name:$(export LC_ALL=C; date +%Y_%m_%d_%H%M)
docker build --force-rm ${DOCKER_ARGS} --tag $image_plus_tag --build-arg USER=$USER --build-arg GROUP=$USER $BUILD_BASE $DIR/vorc-server && \
docker tag $image_plus_tag $image_name:latest && \
echo "Built $image_plus_tag and tagged as $image_name:latest"

set +x
echo -e "${GREEN}Done.${NOCOLOR}\n"
