#!/bin/bash

# generate_trial_video.bash: A bash script to generate a video from a Gazebo
# playback log file.
#
# Input: Gazebo log file state.log
# Output: Recorded video in .ogv
#
# Usage:
# ./generate_trial_video.bash -n example_team station_keeping 0
#
# Please install the following dependencies before using the script:
#   sudo apt-get install recordmydesktop wmctrl psmisc

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Play back the log file.
# Specify --keep-docker so that it doesn't block for container to terminate and
# clean up processes, as we will do that later in this script.
# Specify --manual-play to start playback from this script after screen
# recording is ready to begin.
# Specify --keep-gz to keep Gazebo client window open until screen recording
# stops.
# TODO: leaving --keep-gz out for now, cannot detect playback ended from here
# for some reason, so Docker injection to kill Gazebo is not being issued.
# Letting play_vorc_log.sh terminate Gazebo until this is fixed.
source ${DIR}/replay_trial.bash --keep-docker --manual-play "$@"

echo "Setting up screen recording..."

# Wait and find the Gazebo Window ID.
# Note: May find Gazebo in browser tab, which is wrong. Please close all Gazebo related tabs.
until wmctrl -lp | grep Gazebo > /dev/null
do
  sleep 1
done
GAZEBO_WINDOW_ID=`wmctrl -lp | grep Gazebo | cut -d" " -f 1`

if [ -z "$GAZEBO_WINDOW_ID" ]; then
  echo "Gazebo window not detected. Exiting..."
  sleep 2
  exit 1
fi

# Move Gazebo window to front
wmctrl -i -a ${GAZEBO_WINDOW_ID}

# Adjust the value of this constant if needed to avoid capturing a black
# screen for a long time.
sleep $BLACK_WINDOW_TIME

# Start recording the Gazebo Window.
echo "Recording screen on host..."
recordmydesktop --fps=30 --windowid=${GAZEBO_WINDOW_ID} --no-sound -o $HOST_OUTPUT \
  > $HOST_OUTPUT.record_output.txt 2>&1 &
echo -e "${GREEN}OK${NOCOLOR}"

# Give Gazebo time to finish starting up
sleep 3s

# Unpause the simulation to begin playback
echo "Injecting signal into Docker container to unpause Gazebo..."
# Inject unpause command into Docker container
UNPAUSE_CMD="gz world -p 0"
docker exec -it ${SERVER_CONTAINER_NAME} ${UNPAUSE_CMD}
echo -e "${GREEN}OK${NOCOLOR}"

# TODO: leaving --keep-gz out for now, cannot detect playback ended from here
# for some reason, so Docker injection to kill Gazebo is not being issued.
# Letting play_vorc_log.sh terminate Gazebo until this is fixed.
## Check if gzclient is running
#is_gzclient_running()
#{
#  if docker exec -it ${SERVER_CONTAINER_NAME} pgrep gzclient > /dev/null; then
#    true
#  else
#    false
#  fi
#}
#
## Wait until the gazebo world stats topic (eg. /gazebo/<world>/world_stats)
## tells us that the playback has been paused. This event will trigger the end of
## the recording.
#wait_until_playback_ends()
#{
#  echo "Waiting for playback in container [${SERVER_CONTAINER_NAME}] to end..."
#  gz_world_stats_topic=$(docker exec -it ${SERVER_CONTAINER_NAME} gz topic -l | grep "world_stats")
#
#  # Sleep until Gazebo is paused
#  until docker exec -it ${SERVER_CONTAINER_NAME} gz topic -e "$gz_world_stats_topic" -d 1 -u | grep "paused: true" --quiet
#  do
#
#    # DEBUG
#    docker exec -it ${SERVER_CONTAINER_NAME} gz topic -e "$gz_world_stats_topic" -d 1 -u | grep "paused: true"
#
#    sleep 1
#    if ! is_gzclient_running ; then
#      echo "Detected gzclient no longer running in container"
#      return 0
#    fi
#  done
#  echo -e "${GREEN}OK${NOCOLOR}"
#}
#
#wait_until_playback_ends
#
## Terminate Gazebo only after recording ends, so that we don't record the screen
## content behind the Gazebo window.
#echo "Injecting signal into Docker container to terminate Gazebo..."
#docker exec -it ${SERVER_CONTAINER_NAME} killall -w gzserver gzclient
#echo -e "${GREEN}OK${NOCOLOR}"

# Wait for Docker container to terminate
echo "Waiting for Docker container to terminate..."
wait $SERVER_PID
echo -e "${GREEN}OK${NOCOLOR}"

echo "Encoding video and storing in $OUTPUT before terminating Gazebo..."
killall -w recordmydesktop
echo -e "${GREEN}OK${NOCOLOR}"

echo "Playback ended"
echo "---------------------------------"

# Kill and remove all containers before exit
${DIR}/utils/kill_vorc_containers.bash

exit 0
