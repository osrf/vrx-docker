#!/bin/bash

# run_container.bash: A bash script to build the vrx server image in a container
#
# This script is used to create and run a docker container from an image
# (usually from a built src-cloudsim image).
# The script expects 4 arguments:
# --- 1) The target container name.
# --- 2) The name of the docker image from which to create and run the container.
# --- 3) Optional extra arguments for docker run command. E.g., some extra -v options
# --- 4) An optional command to execute in the run container. E.g. /bin/bash 
#
# Example command line:
# E.g.: ./run_container.bash vrx-server-system vrx-server-noetic "-e ROS_IP=172.XX.XX.XX" "/run_script.sh"    # non-Nvidia
#   or  ./run_container.bash -n vrx-server-system vrx-server-noetic-nvidia "-e ROS_IP=172.XX.XX.XX" "/run_script.sh" # Nvidia
#
# Requires:
#   docker
#   nvidia-docker
#   an X server

set -x

# Parse arguments
RUNTIME="runc"

POSITIONAL=()
while [[ $# -gt 0 ]]
do
  key="$1"
  
  case $key in
      -n|--nvidia)
      RUNTIME="nvidia"
      shift
      ;;
      *)    # unknown option
      POSITIONAL+=("$1")
      shift
      ;;
  esac
done

set -- "${POSITIONAL[@]}"

if [[ $# -lt 2 ]] 
then
    echo "Usage: $0 [-n --nvidia] <container_name> <image_name> [<docker_extra_args> <command>]"
    exit 1
fi

CONTAINER=$1
IMAGE_NAME=$2
DOCKER_EXTRA_ARGS=$3
COMMAND=$4

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

USERID=$(id -u)
GROUPID=$(id -g)

docker run --name ${CONTAINER} \
  -e XAUTHORITY=$XAUTH \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/tmp/.dockerifvefjga.xauth:/tmp/.dockerifvefjga.xauth" \
  -v /dev/log:/dev/log \
  -v "/dev/input:/dev/input" \
  --gpus all \
  --privileged \
  --security-opt seccomp=unconfined \
  -u $USERID:$GROUPID \
  ${DOCKER_EXTRA_ARGS} \
  ${IMAGE_NAME} \
${COMMAND}

