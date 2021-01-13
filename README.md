# VRX Automated Evaluation

This repository contains scripts and infrastructure that will be used for automatic evaluation of Virtual RobotX (VRX) teams' submissions. 

This repository consists of two major components: 

1. The VRX server system, which runs the VRX Gazebo simulation

2. The VRX competitor system, which runs a team's control software.

For security and reproducibility, the VRX server and the VRX competitor's systems will be run in separate, isolated environments called Docker containers.

## Overview

### Scripts

This repository primarily consists of the following main scripts:

* `vrx_server/vrx-server/build_image.bash` - A script that runs `docker build ...` on the files in `vrx_server/vrx-server` to build the image. This script may take 30-60 minutes to run the first time, but is cached afterwards. It must be run every time the files in `vrx_server/vrx-server` are modified to not simply use the old cached versions.

* `prepare_team_wamv.bash` - A script that runs `roslaunch vrx_gazebo generate_wamv.launch ...` and stores the generated files appropriately.

* `prepare_task_trials.bash` - A script that runs `roslaunch vrx_gazebo generated_worlds.launch ...` and stores the generated files appropriately

* `run_trial.bash` - A script that runs a vrx trial with a given team, task and environmental condition. It requires that the above prepare scripts be called before using it. It is the script that runs the simulation and the competitor container. This is the most important script of this repository.

* `vrx_server/vrx-server/run_vrx_trial.sh` - A script that the vrx-server container runs after entry. This is the code that starts the simulation. Please note that you must run the build image script every time you modify this file to see the changes. This script is automatically called when running `run_trial.bash`

* `generate_trial_video.bash` - A script that runs `roslaunch vrx_gazebo playback.launch ...` to playback a trial and also record it with `recordmydesktop`. It requires that the above run script be called before using. This script is not important for the running of the competition, but for visualizing and documenting progress.

Every other script is either:

* a helper script that the main scripts call or

* a multi_script version of the main scripts, which means that it calls the main scripts multiple times for convenience (eg. do multiple tasks, trials, teams, etc.)

The average user should not need to modify any files, but may want to change some values in one of the main scripts. The other scripts should not need to be modified.

### File structure

This section will give descriptions of the directories in this repository:

* `vrx_server` - contains scripts for building and running the vrx-server container, as well as its Docker files

* `team_config` - stores the team config files. The prepare team scripts look inside of this directory to generate the WAM-V URDF files. More details about this below.

* `task_config` - stores the task config files. The prepare task scripts look inside of this directory to generate the trial world files. More details about this below.

* `utils` - contains helper scripts that the main scripts call

* `multi_scripts` - contains convenience scripts that call the main scripts multiple times

* `generated` - contains generated files from all the scripts. This includes command outputs, scores, ROS logs, Gazebo logs, videos, WAM-V URDFs, trial worlds etc. More details about this below.

## Quick Start Instructions For a Single Trial: Setting up workspace to run automated evaluation

### Installing Docker

Docker is required to run the automated evaluation. 
Please follow the [VRX Docker install tutorial](https://github.com/osrf/vrx/wiki/tutorials-installDocker) and the [Nvidia Docker install tutorial](https://github.com/osrf/vrx/wiki/tutorials-installNvidiaDocker) if you are
using an Nvidia GPU.

### Setting up vrx\_gazebo

`vrx_gazebo` must be setup on your machine to run these scripts. As of July 23, 2019, having vrx from source is required (this is related to Issue#1 of this repository). 
If you have not done so already, please follow the [VRX System Setup Tutorial](https://github.com/osrf/vrx/wiki/tutorials-SystemSetupInstall) sections __Install all prerequisites in your host system__ and __Option 2: Build VRX from source__. 
Make sure it is sourced so that you can run launch files from `vrx_gazebo`. Make sure that your file structure is `/home/<username>/vrx_ws`, as this will assure reliable functionality.

### Installing dependencies

* `pip install oyaml` - for generating wam-v and worlds

* `sudo apt-get install recordmydesktop wmctrl psmisc vlc` - for generating and viewing videos

### Setting up vrx-docker

`vrx-docker` must be setup on your machine to run these scripts. Please note that most of the commands listed here are written to be run from the vrx-docker directory. These scripts should be able to be run from other directories, but you will need to change the command paths accordingly.

```
git clone https://github.com/osrf/vrx-docker.git
cd vrx-docker
```

### Building your vrx-server image

The next step is to build your vrx-server image. This involves running

```
./vrx_server/build_image.bash -n # for Nvidia computers

OR

./vrx_server/build_image.bash # for non-Nvidia computers (not recommended, runs slower than real-time.)


# Usage: ./vrx_server/build_image.bash [-n --nvidia]
```

This will create the image for the vrx server that runs the simulation. This step may take 30-60 minutes the first time, but it will be cached in the future calls.

### Adding VRX team files

To run the competition with a VRX team's configuration, the team's folder containing its configuration files must be put into the `team_config` directory.

We have provided example submissions in the `team_config` directory of this repository. 
You should see that there is a directory called `example_team` that has the following configuration files in it:

```
$ ls team_config/example_team/
dockerhub_image.txt sensor_config.yaml thruster_config.yaml
```

Together these files constitute a submission. The files are explained in the __Files Required From VRX Teams For Submission__ section below. 
We will work with the files of the `example_team` submission for this tutorial; you can use them as a template for your own team's submission.

### Preparing a team's system

To prepare a team's system, call:

```
./prepare_team_wamv.bash example_team

# For your team you will run:
# ./prepare_team_wamv.bash <your_team_name>
```

This will call `generate_wamv.launch` on the files in `team_config/example_team` and store the generated files in `generated/team_generated/example_team`.

The end of the output for this command should look something like this:
```
[INFO] [1570212125.006407]: 
Using /home/bsb/vrx_comp/vrx-docker/team_config/example_team/sensor_config.yaml as the sensor configuration yaml file


WAM-V urdf file sucessfully generated. File location: /home/bsb/vrx_comp/vrx-docker/generated/team_generated/example_team/example_team.urdf
================================================================================REQUIRED process [wamv_config/wamv_generator-2] has died!
process has finished cleanly
log file: /home/bsb/.ros/log/123992a6-e6d1-11e9-af7a-408d5c555554/wamv_config-wamv_generator-2*.log
Initiating shutdown!
================================================================================
[wamv_config/wamv_generator-2] killing on exit
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
OK

Checking /home/bsb/.ros/log/latest/wamv_config-wamv_generator-2.log for compliance
Compliant? true. Writing to /home/bsb/vrx_comp/vrx-docker/generated/team_generated/example_team/compliant.txt
OK
```
Note that the `REQUIRED process [wamv_config/wamv_generator-2] has died!` message is expected.

This will also create a file `generated/team_generated/example_team/compliant.txt` that says `true` is the configuration was compliant or `false` otherwise. By compliant, we are refering to the limitations for sensor and propulsion configuration described in the [VRX competition documentation](https://github.com/osrf/vrx/wiki/documentation) .You can view this result by:

```
cat generated/team_generated/example_team/compliant.txt
```

### Preparing trials for a task

In this README, we will be using some vocabulary that will be clearly defined here.

* `task`: One of the major competition tasks, e.g., `stationkeeping`, `wayfinding`, `perception`, `nav_challenge`, `dock` or `scan_and_dock`.

* `trial`: A specific task with a specific set of environmental conditions (e.g., sea state, wind magnitude and direction, lighting, etc.). Each task has multiple trials. Each trial will have a specific world file associated with it.

* `trial_number`: Each task will have `n` trials. Each trial will be labelled with a trial\_number, which ranges from `0` to `n-1` inclusive.

To prepare all of the trials for a task, call:

```
./prepare_task_trials.bash stationkeeping

#./prepare_task_trials.bash <task_name>
```

This will call `generate_worlds.launch` on `task_config/stationkeeping.yaml` and store the generated files in `generated/task_generated/stationkeeping`.

The last part of the output of this should look something 
```
setting /run_id to 172c61c0-e6cd-11e9-af7a-408d5c555554
process[rosout-1]: started with pid [30491]
started core service [/rosout]
process[world_gen-2]: started with pid [30505]



All  3  worlds generated
================================================================================REQUIRED process [world_gen-2] has died!
process has finished cleanly
log file: /home/bsb/.ros/log/172c61c0-e6cd-11e9-af7a-408d5c555554/world_gen-2*.log
Initiating shutdown!
================================================================================
[world_gen-2] killing on exit
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
OK
```
where the important parts are `All  3  worlds generated` indicating success.  Also note that the `REQUIRED process [world_gen-2] has died!` message is expected.


Please note that we will be writing our own private .yaml files for the tasks. Essentially, the only difference between testing out your system with these steps and the real competition is that for the real competition, we will be creating our own `.yaml` files for tasks that you have not seen, which will vary the environmental conditions. We will not be trying to surprise you with the conditions, but we want to simply reward teams that are robust to different environmental conditions.

### File structure after successful prepare

After running the prepare scripts, you should have the following file structure in `generated/team_generated`:

```
generated/team_generated
├── example_team
│   ├── compliant.txt
│   ├── example_team.urdf
│   ├── sensor_config.xacro
│   └── thruster_config.xacro
```

After running the prepare scripts, you should have the following file structure in `generated/task_generated`:

```
generated/task_generated
├── stationkeeping
│   ├── worlds
│   │   ├── stationkeeping0.world
│   │   └── stationkeeping1.world
│   └── world_xacros
│       ├── stationkeeping0.world.xacro
│       └── stationkeeping1.world.xacro
```

If you are missing these files, please review the command output (either in terminal or in `multi_scripts/prepare_output/`) to investigate the issue.

## Quick Start Instructions For a Single Trial: Running a single trial for a single team

In order to run a trial with a specific team, the prepare scripts above must have been called on the associated task and team before running. To run a single trial with a specific team (in this case the team from`team_config/example_team` and the trial with trial\_number 0 associated with `task_config/stationkeeping.yaml`), call:

```
./run_trial.bash -n example_team stationkeeping 0 # for Nvidia computers
# OR
./run_trial.bash example_team stationkeeping 0 # for non-Nvidia computers


# For your team you will run:
# ./run_trial.bash [-n --nvidia] <your_team_name> <task_name> <trial_number>
```

This will instantiate two Docker containers.

1. The simulation server container, which runs the VRX Gazebo simulation with the desired team WAM-V and desired task trial world.

2. The VRX team's container, which runs their system from the Dockerhub image in `team_config/<your_team_name>/dockerhub_image.txt`.

After the trial is over, it stores log files of the results. More about logs in a section below.

This pulls the competitor image first, then starts the simulation container, then starts the competitor container. Note: Need to ensure that there is enough initial_state_duration time so that competition does not start before the competitor container has started. 

## Quick Start Instructions For a Single Trial: Reviewing the results of a trial

### Reviewing the trial performance

After the trial has finished, you can go to `generated/logs/<your_team_name>/<task_name>/<trial_number>/` to review the generated log files. 

The `generated/logs` directory has the following structure:

```
generated/logs
└── example_team
    ├── stationkeeping
    │   └── 0
    │       ├── gazebo-server
    │       │   ├── ogre.log
    │       │   ├── server-11345
    │       │   │   ├── default.log
    │       │   │   └── gzserver.log
    │       │   └── state.log
    │       ├── ros-competitor
    │       │   ├── rostopic_29_1564524270930.log
    │       │   └── rostopic_30_1564524270921.log
    │       ├── ros-server-latest
    │       │   ├── master.log
    │       │   ├── roslaunch-d560550c9290-50.log
    │       │   ├── rosout-1-stdout.log
    │       │   ├── rosout.log
    │       │   ├── spawn_model-3.log
    │       │   └── spawn_model-3-stdout.log
    │       ├── trial_score.txt
    │       ├── verbose_output.txt
    │       └── vrx_rostopics.bag
```

The `generated/logs` directory will have numerous directories with the date and time of creation. In each of those directories are the log files of each trial.

* gazebo-server - contains the gazebo server logs

* ros-competitor - the log files from the competitor's container. Note: this is more prone to error, as finding the files depends on the competitor images's file structure

* ros-server-latest - contains the log files from the ros server

* vrx_rostopics.bag - a bag file containing rostopics from the vrx trial. Currently only stores `/vrx/task/info` to save space, but this can be edited in `vrx_server/vrx-server/run_vrx_trial.sh`. Note: to apply any changes to this file, you must also run ./vrx_server/build_image.bash to use the updated file, instead of cache.

* verbose_output.txt - verbose Gazebo log output from the competition

* trial_score.txt - text file with one number representing the final score of the trial (from the last message of vrx_rostopics.bag)

* task_score.txt - (only created when running multi_scripts, how to do so in section below) text file with comma separated values, which are the trial scores of one task for a given team

* team_score.txt - (only created when running multi_scripts, how to do so in section below) text file with comma separated values, which are the trial scores of all tasks for a given team

* video - (only created after running generate video scripts, how to do so in section below) contains the generated trial video and its record and playback command outputs

View the score of the trial by running

```
cat generated/logs/example_team/stationkeeping/0/trial_score.txt
```

## Quick Start Instructions For a Single Trial: Trial videos and playback

### Playing back the recorded log of a single trial

After running a trial, a `state.log` file is stored under `generated/logs/<team>/<task>/<trial_num>/gazebo-server`. This is a playback log file that allows you to play back the trial. 

To play back the trial with Gazebo GUI from within the competition Docker container, run

```
./replay_trial.bash -n example_team stationkeeping 0
```

By default, this is on autopilot and will shut down Gazebo after log playback is finished.
A side effect is that it will shut down if the playback is paused (which happens to be an indication of the log playback having finished).

To inspect more carefully, you may wish to pause playback without Gazebo shutting down.
That can be done by issuing `--keep-gz` on the command line:
```
./replay_trial.bash -n --keep-gz example_team wayfinding 0
```

When you are done with manual inspection, you can shut down Gazebo cleanly by issuing this to the container:
```
docker exec -it vrx-server-system killall -w gzclient gzserver
```
The autopilot will detect Gazebo termination and shut down the rest of the container.

`replay_trial.bash` accepts a few options for more manual inspections, which you can view in the usage by running it alone without arguments.

### Generating a single trial video

To make a screen recording of the trial while playing back, run the following command. Internally, this calls `replay_trial.bash` above.

```
./generate_trial_video.bash -n example_team stationkeeping 0

# For your team you will run:
# ./generate_trial_video.bash -n <your_team_name> <task_name> <trial_number>
```

This will start the Gazebo trial playback, begin screen capture on the Gazebo window, and then store the video file, record output and playback output in `generated/logs/<team>/<task>/<trial_num>/video`. 
Please note that you must close other tabs related to Gazebo for this to work properly, as it puts the Gazebo window at the front (not background). Keep the Gazebo window on top. If you have a browser tab open related to Gazebo,
it may find that window, instead of the actual Gazebo simulation window.

There should be a new directory called `generated/logs/<team>/<task>/<trial_num>/video` that contains the following:

```
generated/logs/example_team/stationkeeping/0/video/
├── playback_video.ogv
├── playback_video.ogv.playback_output.txt
└── playback_video.ogv.record_output.txt
```

You can play the video with

```
vlc generated/logs/example_team/stationkeeping/0/video/playback_video.ogv
```

### Playing back the simulation

To play back a specific trial's log file, move to `vrx-docker` and call:

```
roslaunch vrx_gazebo playback.launch log_file:=`pwd`/generated/logs/example_team/stationkeeping/0/gazebo-server/state.log

# For your team you will run:
# roslaunch vrx_gazebo playback.launch log_file:=`pwd`/generated/logs/<your_team_name>/<task_name>/<trial_number>/gazebo-server/state.log
```

## Important information

* All generated files will be stored in the `generated` directory. This will include team and task generated files, log files, scoring, playback videos, etc. These files may get overwritten if scripts are called more than once. Remember to delete these generated files if you want to start fresh.

* After calling `./vrx_server/build_image.bash` the first time, your image will be cached. This means that it will use the old image until this script is called again. If you update `vrx_server/vrx-server/run_vrx_trial.sh` or any file in `vrx_server/vrx-server`, those changes will not affect things until you call `./vrx_server/build_image.bash` again after making the change

* For video generation, you can edit `generate_trial_video.bash` to change the `x, y, width, height, or BLACK_WINDOW_TIME` variables to change the position and size of recording as well as the length of time that is waited before recording starts.

* Currently, only the `/vrx/task/info` topic is recorded in the generated rosbag to save space. You can change this by editing `vrx_server/vrx-server/run_vrx_trial.sh` and changing the `rosbag record ...` line to `rosbag record -O ~/vrx_rostopics.bag --all &`

* Currently, the playback log files are always played back at the same speed. To change the speed of playback, you must change the rate of recording. You can change this by editing `vrx_server/vrx-server/run_vrx_trial.sh` and changing the `RECORD_PERIOD=...`. A higher number means faster playback. A lower number means slower playback. 

* Note that some scripts (build_image.bash, run_container.bash, run_trial.bash, all multi_scripts for running) have an optional \[-n --nvidia\] argument that should be set for Nvidia devices. Please create an issue if there are any difficulties with this.

## Expected errors:

* In `verbose_output.txt`, expect to see 

```
Error [parser_urdf.cc:3170] Unable to call parseURDF on robot model
Error [parser.cc:406] parse as old deprecated model file failed.
...
...
[Msg] OnReady
[Msg] OnRunning
[Msg] 4.999000Segmentation fault (core dumped)
[gazebo-2] process has died [pid 86, exit code 139, cmd /opt/ros/melodic/lib/gazebo_ros/gzserver --verbose -e ode -r --record_period 0.01 --record_path /home/tylerlum/.gazebo /task_generated/worlds/perception0.world __name:=gazebo __log:=/home/tylerlum/.ros/log/1d527252-b319-11e9-89c0-0242ac100016/gazebo-2.log].
log file: /home/tylerlum/.ros/log/1d527252-b319-11e9-89c0-0242ac100016/gazebo-2*.log

```

The parse error message comes from recording, and is a known issue that does not affect the competition. The segmentation fault at the end comes from the scoring plugin shutting down Gazebo when the competition is over.

* A known bug is that getting log files from the competitor container might not work, depending on the location that it is stored.

* As well, during video generation you cannot have any other windows related to Gazebo open. The script looks for a window with Gazebo in the name to move to the front for recooding, but this can be ruined by another window.

* When building the vrx server image with `./vrx_server/build_image.bash [-n --nvidia]`, you can expect to see the following types errors, which are normal:

```
WARNING: apt does not have a stable CLI interface…
```

* Depending on the size of your display, the resolution/size of the video recording may be incorrect. If your video generation does not work, type the following command and see the result.

```
cat generated/logs/<team>/<task>/<trial_num>/video/playback_video.ogv.record_output.txt

Window size specification out of bounds!(current resolution:1920x1080)
```

You can look in the __Important Information__ section for details about how to change the width and height.

## Multi_scripts

The above quick start instructions gave an overview of how to use the main scripts to run automated evaluation. For convenience, we also have multi_scripts that run the main scripts multiple times for convenience. We describe these below.

Before beginning, you may want to remove the previously generated files to start fresh, as it may get confusing or messy if you keep old files along with new ones.

Note that some of the files are tracked in the Git repository, so you may want to rename rather than remove, or plan on stashing and dropping the removals afterwards.

```
rm -r generated
```

### Prepare all scripts

For the purpose of competition, we have a convenience script to run prepare all teams and a convenience script to run prepare all tasks.

Prepare all teams:
```
./multi_scripts/prepare_all_team_wamvs.bash

# Runs ./prepare_team_wamv.bash on all teams in team_config
```

Prepare all tasks:
```
./multi_scripts/prepare_all_task_trials.bash

# Runs ./prepare_task_trials.bash on all task yaml files in task_config
```

To keep the terminal output clean, all of the output will be stored in `generated/multi_scripts/prepare_output/`. These scripts should end if there is an error and show `OK` if it is working. These convenience scripts are more bug-prone, so if you notice any issues, please submit an issue [here](https://github.com/osrf/vrx-docker/issues).

### Running all trials for a given task for a single team

To run all trials for a given task, call:

```
./multi_scripts/run_one_team_one_task.bash example_team example_task # for non-Nvidia computers
# or
./multi_scripts/run_one_team_one_task.bash -n example_team example_task # for Nvidia computers

# For your team you will run:
# ./multi_scripts/run_one_team_one_task.bash [-n --nvidia] <your_team_name> <task_name>
```

This will run `run_trial.bash` on all trials in `generated/task_generated/<task>/worlds` for a given task for one team.

### Running all trials for all tasks for a single team

To run all trials for all tasks listed in the `task_generated` directory, call:

```
./multi_scripts/run_one_team_all_tasks.bash example_team # for non-Nvidia computers
# or
./multi_scripts/run_one_team_all_tasks.bash -n example_team # for Nvidia computers

# For your team you will run:
# ./multi_scripts/run_one_team_all_tasks.bash [-n --nvidia] <your_team_name>
```

This will run `run_one_team_one_task.bash` on all tasks in `generated/task_generated/` for a given team.

### Running all trials for all tasks for all teams

To run all trials for all tasks listed in the `task_generated` directory for all teams in `team_generated`, call:

```
./multi_scripts/run_all_teams_all_tasks.bash # for non-Nvidia computers
# or
./multi_scripts/run_all_teams_all_tasks.bash -n # for Nvidia computers

# For your team you will run:
# ./multi_scripts/run_all_teams_all_tasks.bash [-n --nvidia]
```

This will run `run_one_team_all_tasks.bash` on all teams in `generated/team_generated`. This is the invocation that will be used to test submissions for the Finals: your system will not be provided with any information about the conditions of the trials. If your system performs correctly with this invocation, regardless of the set of configuration files in the `task_config` directory, you're ready for the competition.

Note: To keep the terminal output clean, all of the output from `multi_scripts` will be stored in `generated/multi_scripts/run_output/`. These convenience scripts are more bug-prone, so if you notice any issues, please submit an issue [here](https://github.com/osrf/vrx-docker/issues).


### Generating all trial videos for a given task for a single team

To generate all trial videos for one team and one task, run

```
./multi_scripts/generate_one_team_one_task_videos.bash example_team example_task

# For your team you will run:
# ./multi_scripts/generate_one_team_one_task_videos.bash <your_team_name> <task_name>
```

This will run `generate_trial_video.bash` for all trials in `generated/logs/<team>/<task>` for a given task and team.

### Generating all trial videos for all tasks for a single team
To generate all trial videos for one team and all its tasks, run

```
./multi_scripts/generate_one_team_all_task_videos.bash example_team

# For your team you will run:
# ./multi_scripts/generate_one_team_all_task_videos.bash <your_team_name>
```

This will run `generate_one_team_one_task_videos.bash` for all tasks in `generated/logs/<team>` for a given team.

### Generating all trial videos for all tasks for all teams
To generate all trial videos for all teams and all its tasks, run

```
./multi_scripts/generate_all_team_all_task_videos.bash

# For your team you will run:
# ./multi_scripts/generate_all_team_all_task_videos.bash
```

This will run `generate_one_team_all_task_videos.bash` for all teams in `generated/logs`.

Note: To keep the terminal output clean, all of the output from multi_scripts will be stored in `generated/multi_scripts/generate_video_output/`. These convenience scripts are more bug-prone, so if you notice any issues, please submit an issue [here](https://github.com/osrf/vrx-docker/issues).

### Expected Output After Multi_scripts

If you are confident your setup is working, you can run

```
./vrx_server/build_image.bash && ./multi_scripts/prepare_all_team_wamvs.bash && ./multi_scripts/prepare_all_task_trials.bash && ./multi_scripts/run_all_teams_all_tasks.bash && ./multi_scripts/generate_all_team_all_task_videos.bash
```

After running this, you should expect your `generated` directory to look like

```
ls generated
logs  multi_scripts  task_generated  team_generated
```

and your `generated/logs` directory to look like 

```
generated/logs
├── example_team
│   ├── stationkeeping
│   │   ├── 0
│   │   │   ├── gazebo-server
│   │   │   │   ├── ogre.log
│   │   │   │   ├── server-11345
│   │   │   │   │   ├── default.log
│   │   │   │   │   └── gzserver.log
│   │   │   │   └── state.log
│   │   │   ├── ros-competitor
│   │   │   │   ├── rostopic_29_1564519107303.log
│   │   │   │   └── rostopic_30_1564519107315.log
│   │   │   ├── ros-server-latest
│   │   │   │   ├── master.log
│   │   │   │   ├── roslaunch-8fe010b975a0-50.log
│   │   │   │   ├── rosout-1-stdout.log
│   │   │   │   ├── rosout.log
│   │   │   │   ├── spawn_model-3.log
│   │   │   │   └── spawn_model-3-stdout.log
│   │   │   ├── trial_score.txt
│   │   │   ├── verbose_output.txt
│   │   │   ├── video
│   │   │   │   ├── playback_video.ogv
│   │   │   │   ├── playback_video.ogv.playback_output.txt
│   │   │   │   └── playback_video.ogv.record_output.txt
│   │   │   └── vrx_rostopics.bag
│   │   ├── 1
│   │   │   ├── gazebo-server
│   │   │   │   ├── ogre.log
│   │   │   │   ├── server-11345
│   │   │   │   │   ├── default.log
│   │   │   │   │   └── gzserver.log
│   │   │   │   └── state.log
│   │   │   ├── ros-competitor
│   │   │   │   ├── rostopic_29_1564519155778.log
│   │   │   │   └── rostopic_30_1564519155773.log
│   │   │   ├── ros-server-latest
│   │   │   │   ├── master.log
│   │   │   │   ├── roslaunch-8410d47cfdaa-51.log
│   │   │   │   ├── rosout-1-stdout.log
│   │   │   │   ├── rosout.log
│   │   │   │   ├── spawn_model-3.log
│   │   │   │   └── spawn_model-3-stdout.log
│   │   │   ├── trial_score.txt
│   │   │   ├── verbose_output.txt
│   │   │   ├── video
│   │   │   │   ├── playback_video.ogv
│   │   │   │   ├── playback_video.ogv.playback_output.txt
│   │   │   │   └── playback_video.ogv.record_output.txt
│   │   │   └── vrx_rostopics.bag
│   │   └── task_score.txt
│   │   ....
│   └── team_score.txt
│   ...
```

## Development tips

### Investigating errors

If you encountered errors, it is recommended that you view the Gazebo verbose output by running:

```
cat generated/logs/<team>/<task>/<trial_num>/verbose_output.txt
```

or investigate the generated rosbag by running:

```
rosbag info generated/logs/<team>/<task>/<trial_num>/vrx_rostopics.bag
```

or if you ran a multi_script, run

```
cat generated/multi_scripts/prepare_output/<team or task>/output.txt
# or
cat generated/multi_scripts/run_output/<team>/<task>/<trial_number>/output.txt
# or 
cat generated/multi_scripts/generate_video_output/<team>/<task>/<trial_number>/output.txt
```

or if you had an issue with video generation you can run

```
cat generated/logs/<team>/<task>/<trial_num>/video/playback_video.ogv.record_output.txt 
# or
cat generated/logs/<team>/<task>/<trial_num>/video/playback_video.ogv.playback_output.txt 
```

You can even run these commands while the script is still running to investigate issues.

### Stopping the Docker containers

If during your development you need to kill the server and competitor containers, you can do so with:

```
./utils/kill_vrx_containers.bash
```

This will kill and remove all VRX containers.

### Investigating issues in the competitor container

If you are having difficulties running your team's system, you can open a terminal in the container that has your system installed. There are two ways:

To create a new container for investigation, run:

```
docker run -it --rm --name vrx-competitor-system <image_name>
```

To investigate a running container for investigation, run:

```
docker exec -it vrx-competitor-system bash
```


From here, you can investigate what is happening inside of your container.

## Submission Details

### Files Required From VRX Teams For Submission

All VRX teams must submit one folder containing three files for automated evaluation. The name of the folder should be the name of the team. Please note that the filenames must be identical with how they are listed below.

1. `sensor_config.yaml`: The team's sensor configuration yaml file. One sensor configuration is used for all trials. For more information about this file, please refer to the [Creating a Custom WAM-V](https://github.com/osrf/vrx/wiki/tutorials-Creating%20a%20custom%20WAM-V%20Thruster%20and%20Sensor%20Configuration%20For%20Competition) tutorial.

2. `thruster_config.yaml`: The team's thruster configuration yaml file. One thruster configuration is used for all trials. For more information about this file, please refer to the [Creating a Custom WAM-V](https://github.com/osrf/vrx/wiki/tutorials-Creating%20a%20custom%20WAM-V%20Thruster%20and%20Sensor%20Configuration%20For%20Competition) tutorial.

3. `dockerhub_image.txt`: A text file containing only the name of their docker image publicly available on Dockerhub, e.g., `tylerlum/vrx-competitor-example:v2.2019`. For more information about this file, please refer to the [Creating a Dockerhub image for submission](https://github.com/osrf/vrx/wiki/tutorials-Creating%20a%20Dockerhub%20image%20for%20submission)

### Testing Your Submission

All teams should test their submissions by following the instructions above. They detail how to run the scripts to test your system in a mock competition.

It is imperative that teams use the outlined process for testing their systems, as it replicates the process that will be used during automated evaluation. If your system does not work in the mock competition setup, then it will not work for the real competition.

### Uploading Your Submission

Details about the submission will be coming shortly.
