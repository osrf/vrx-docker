#!/bin/bash

# gz_utils.sh: Utility functions used for running and playing back trials.

function is_gzserver_running()
{
  if pgrep gzserver > /dev/null; then
    true
  else
    false
  fi
}

# Check if gzclient is running
function is_gzclient_running()
{
  if pgrep gzclient > /dev/null; then
    true
  else
    false
  fi
}

function wait_until_gzserver_is_down()
{
  until ! is_gzserver_running
  do
    sleep 1
  done
}

function wait_until_gzserver_is_up()
{
  until is_gzserver_running
  do
    sleep 1
  done
}
