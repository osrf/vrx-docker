#!/bin/bash

# prepare_team_wamv.bash: A bash script to generate single team's WAM-V urdf file.
#
# E.g.: ./prepare_team_wamv.bash example_team

set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define compliance check function.

is_wamv_compliant()
{
  # Get logs
  log_dir=$HOME/.ros/log
  logs=$(ls -t $log_dir)

  # Find latest wamv_generator log
  for log in $logs; do
      if [[ $log == *"wamv_generator"* ]]; then
        break
      fi
  done

  echo "Checking $log_dir/$log for compliance"

  # Check if the log file has errors
  if grep -Fq "ERROR" $log_dir/$log
  then
      return 1
  else
      return 0
  fi
}

# Define usage function.
usage()
{
  echo "Usage: $0 <team>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 1 ]] && usage

TEAM_NAME=$1

echo -e "${GREEN}Preparing WAM-V URDF for team: ${TEAM_NAME}${NOCOLOR}"

# Get directory of this file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Remove previous log files, so new log data can be seen more easily
# (currently done for log check for compliance)
log_dir=$HOME/.ros/log/latest
# Add trailing slash so that we remove the contents of the linked directory
rm -fR $log_dir/

# Find component and thruster yaml files
echo "Looking for config files"
TEAM_CONFIG_DIR=${DIR}/team_config/${TEAM_NAME}
COMPONENT_CONFIG=${TEAM_CONFIG_DIR}/component_config.yaml
THRUSTER_CONFIG=${TEAM_CONFIG_DIR}/thruster_config.yaml

if [ -f "${COMPONENT_CONFIG}" ]; then
  echo -e "Successfully found: ${COMPONENT_CONFIG}"
else
  echo -e "${RED}Err: ${COMPONENT_CONFIG}"; exit 1;
fi
if [ -f "${THRUSTER_CONFIG}" ]; then
  echo -e "Successfully found: ${THRUSTER_CONFIG}\n"
else
  echo -e "${RED}Err: ${THRUSTER_CONFIG}"; exit 1;
fi

# Create directory for generated files
wamv_target_dir=${DIR}/generated/team_generated/${TEAM_NAME}
wamv_target=${wamv_target_dir}/${TEAM_NAME}.urdf
mkdir -p ${wamv_target_dir}

# Generate WAM-V
echo "Generating WAM-V..."
ros2 launch vrx_gazebo generate_wamv.launch.py component_yaml:=$COMPONENT_CONFIG thruster_yaml:=$THRUSTER_CONFIG wamv_target:=$wamv_target wamv_locked:=true
echo -e "${GREEN}OK${NOCOLOR}\n"

# Write to text file about compliance
if is_wamv_compliant; then
  is_compliant=true
else
  is_compliant=false
fi

echo "Compliant? $is_compliant. Writing to ${wamv_target_dir}/compliant.txt"
echo $is_compliant > ${wamv_target_dir}/compliant.txt
echo -e "${GREEN}OK${NOCOLOR}\n"

# Move generated files to correct location
mv ${TEAM_CONFIG_DIR}/component_config.xacro ${wamv_target_dir}
mv ${TEAM_CONFIG_DIR}/thruster_config.xacro ${wamv_target_dir}
