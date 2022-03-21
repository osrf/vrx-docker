#!/bin/bash

# Create ros master if not already started
rostopic list > /dev/null 2>&1
retVal=$?
if [ $retVal -ne 0 ]; then
    roscore &
    echo "Wait for 5s to allow rosmaster to start"
    sleep 5s
else
    echo "rosmaster already setup"
fi

# Send forward command
RATE=1
CMD=2
echo "Sending forward command"
rostopic pub /left_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD} &
rostopic pub /right_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD}
