#!/bin/bash

is_gzserver_running()
{
  if pgrep gzserver >/dev/null; then
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
