#!/bin/bash

# Create ros master if not already started
rostopic list > /dev/null 2>&1
retVal=$?
if [ $retVal -ne 0 ]; then
	roscore&
	echo "Wait 5s for roscore"
	sleep 5s
else
	echo "rosmaster already running"
fi

source /vrx_ws/devel/setup.bash

python3 basic_node.py
