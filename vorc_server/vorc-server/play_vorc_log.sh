#!/bin/bash

# play_vorc_log.sh: A shell script to play back Gazebo log file from a VORC
# trial.

# Get directory of this file
#DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

#source ${DIR}/gz_utils.sh

is_gzserver_running()
{
  if pgrep gzserver > /dev/null; then
    true
  else
    false
  fi
}

# Check if gzclient is running
is_gzclient_running()
{
  if pgrep gzclient > /dev/null; then
    true
  else
    false
  fi
}

wait_until_gzserver_is_down()
{
  until ! is_gzserver_running
  do
    sleep 1
  done
}

wait_until_gzserver_is_up()
{
  until is_gzserver_running
  do
    sleep 1
  done
}

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <log_file> <destination_file> [--keep-gz]"
  echo "--manual-play: Do not automatically start playback. Wait for user to click in GUI."
  echo "  By default, playack automatically starts."
  echo "--keep-gz: Keep Gazebo server and client running after playback ends."
  echo "  By default, they are shut down automatically."
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 2 ]] && usage

autoplay=1
kill_gz=1

POSITIONAL=()
while [[ $# -gt 0 ]]
do
  key="$1"

  case $key in
    --manual-play)
      autoplay=0
      shift
      ;;

    --keep-gz)
      kill_gz=0
      shift
      ;;

    # Treat unknown options as positional args
    *)
      POSITIONAL+=("$1")
      shift
      ;;
  esac
done

set -- "${POSITIONAL[@]}"

LOG_FILE=$1
OUTPUT=$2

# Create a directory for the Gazebo log and the score file.
DESTINATION_FOLDER="$(dirname "$OUTPUT")"
if [ -d "$DESTINATION_FOLDER" ]; then
  echo -e "${YELLOW}Wrn: Destination folder already exists. Data might be"\
          "overwritten${NOCOLOR}\n"
fi
mkdir -p $DESTINATION_FOLDER

echo "Starting Gazebo..."
echo "Check any possible errors after the run in $OUTPUT.playback_output.txt..."

# Start Gazebo in playback mode
roslaunch vorc_gazebo playback.launch gui:=true log_file:=$LOG_FILE paused:=true verbose:=true \
  > $OUTPUT.playback_output.txt 2>&1 &
roslaunch_pid=$!
wait_until_gzserver_is_up
echo -e "${GREEN}OK${NOCOLOR}\n"

echo "Waiting for server to start up"
sleep 9s

# Check if the log file has errors, likely forgot to source ws
if grep -Fq "RLException" $OUTPUT.playback_output.txt
then
  echo "Failed to find playback launch file. Did you source your vorc_ws?"
  exit 1
fi

echo "Playing back log file..."

wait_for_unpause_signal()
{
  echo "Waiting for unpause signal or manual unpause..."
  gz_world_stats_topic=$(gz topic -l | grep "world_stats")

  # Source ROS
  source $HOME/vorc_ws/install/setup.bash

  # Wait for Docker injection to unpause
  until gz topic -e "$gz_world_stats_topic" -d 1 -u | grep "paused: false" --quiet
  do
    # If user manually closed Gazebo, no need to wait for host signal
    if ! is_gzclient_running || ! is_gzserver_running ; then
      echo "Detected gzserver or gzclient no longer running"
      break
    fi

    sleep 1
  done

  echo -e "${GREEN}OK${NOCOLOR}"
}

if [ $autoplay -eq 1 ]; then
  echo "Unpausing to start playback..."
  gz world -p 0
  echo -e "${GREEN}OK${NOCOLOR}"
else
  wait_for_unpause_signal
fi
# Sleep a little before start checking for pause status, to give Gazebo time
# to unpause. Otherwise an oudated pause is detected immediately.
sleep 1s

# Wait until the gazebo world stats topic (eg. /gazebo/<world>/world_stats)
# tells us that the playback has been paused. This event will trigger the end of
# the recording.
wait_until_playback_ends()
{
  echo "Waiting for playback to end..."
  gz_world_stats_topic=$(gz topic -l | grep "world_stats")

  # Sleep until Gazebo is paused
  until gz topic -e "$gz_world_stats_topic" -d 1 -u | grep "paused: true" --quiet
  do
    sleep 1
    if ! is_gzclient_running ; then
      echo 1>&2 "Detected gzclient no longer running"
      return 0
    fi
  done
  echo -e "${GREEN}OK${NOCOLOR}"
}

wait_until_playback_ends

if [ $kill_gz -eq 1 ]; then
  killall -w gzserver gzclient
else
  echo "Waiting for Docker injection to terminate Gazebo..."
fi
wait_until_gzserver_is_down

# Kill rosnodes
echo "Killing rosnodes"
rosnode kill --all
sleep 1s
echo -e "${GREEN}OK${NOCOLOR}\n"

# Kill roslaunch
echo "Killing roslaunch pid: ${roslaunch_pid}"
kill -INT ${roslaunch_pid}
echo -e "${GREEN}OK${NOCOLOR}\n"
