debug_notes_bsb.md


Notes for debugging vrx-docker functionality.


# Overview of running a trial.

As explained in the README.md

```
# (re)Build server image
./vrx_server/build_image.bash -n # for Nvidia computers

# Prepare team system
./prepare_team_wamv.bash example_team

# Prepare tasks
tasks=(stationkeeping wayfinding perception nav_challenge dock scan_and_dock)
for task in "${tasks[@]}"
do
  ./prepare_task_trials.sh $task
done

# Run server and team containers
./run_trial.bash -n example_team stationkeeping 0 
```

# Turn on Gazebo client GUI

1. In `vrx-docker/vrx_server/vrx-server/run_vrx_trial.sh` find the roslaunch command (~ Line 65) and change `gui:=false` to `gui:=true`
2. Rebuild the server `./vrx_server/build_image.bash -n # for Nvidia computers`


# Running server forever 

Continue (-c) option sets VRX exit on completion to false
```
./run_trial.bash -n -c example_team stationkeeping 0 
```

# Inspecting the system

After starting a trial, you should have two containers - 
```
bsb@aku:~/vrx_comp/vrx-docker$ docker ps
CONTAINER ID        IMAGE                                     COMMAND                  CREATED             STATUS              PORTS               NAMES
c7f74aa2cac6        tylerlum/vrx-competitor-example:v2.2019   "/ros_entrypoint.sh …"   16 seconds ago      Up 14 seconds                           vrx-competitor-system
a69b0cca7dc9        vrx-server-melodic-nvidia:latest          "/vrx_entrypoint.sh …"   25 seconds ago      Up 24 seconds       11345/tcp           vrx-server-system
```

## Inspecting from host

Set env variables for ROS
```
source ~/vrx_ws/devel/setup.bash
export SERVER_ROS_IP="172.16.0.22"
export ROS_MASTER_URI="http://${SERVER_ROS_IP}:11311"
export ROS_IP="172.16.0.1"
```

Then you should be able to view ROS traffic, e.g., 
```
rostopic list
rostopic echo /vrx/task/info 
rosrun rqt_graph rqt_graph
```

## Inspect from vrx-server-system container

Join the server container
```
~/vrx_ws/src/vrx/docker/join.bash vrx-server-melodic-nvidia
```

## Inspect from team container

The default join script doesn't seem to work b/c is strips out the tylerlum bit with basename

```
IMG="tylerlum/vrx-competitor-example:v2.2019"
containerid=$(docker ps -aqf "ancestor=${IMG}")
docker exec --privileged -e DISPLAY=${DISPLAY} -e LINES=`tput lines` -it ${containerid} bash
```

Then
```
source /opt/ros/melodic/setup.bash 
rostopic list
```

# Shutting down

```
./utils/kill_vrx_containers.bash 
```