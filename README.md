# VRX Automated Evaluation

This repository contains scripts and infrastructure that will be used for automatic evaluation of Virtual RobotX (VRX) teams' submissions. 

## Overview

This repository consists of two major components: 

1. The VRX server system, which runs the VRX Gazebo simulation

2. The VRX competitor system, which runs a team's control software.

For security and reproducibility, the VRX server and the VRX competitor's systems will be run in separate, isolated environments called Docker containers.

## Setting up workspace to run automated evaluation

### Installing Docker

Docker is required to run the automated evaluation. Please follow the [Docker CE for Ubuntu tutorial's](https://docs.docker.com/install/linux/docker-ce/ubuntu) __Prerequisites__ and __Install Docker CE__ sections.

Then, continue to the [post-install instructions](https://docs.docker.com/engine/installation/linux/linux-postinstall/) and complete the __Manage Docker as a non-root user__ section to avoid having to run the commands on this page using `sudo`.

### Setting up vrx\_gazebo

`vrx_gazebo` must be setup on your machine to run these scripts. As of July 23, 2019, having vrx from source is required (this is related to Issue#1 of this repository). Please, follow the [VRX System Setup Tutorial](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall) sections __Install all prerequisites in your host system__ and __Option 2: Build VRX from source__. Make sure it is sourced so that you can run launch files from `vrx_gazebo`.

### Adding VRX team files

To run the competition with a VRX team's configuration, the team's folder containing its configuration files must be put into the `team_config` directory.

We have provided an example submission in the `team_config` directory of this repository. You should see that there is a directory called `example_team` that has the following configuration files in it:

```
$ ls team_config/example_team/
dockerhub_image.txt sensor_config.yaml thruster_config.yaml
```

Together these files constitute a submission. The files are explained in the __Files Required From VRX Teams For Submission__ section below. We will work with the files of the `example_team` submission for this tutorial; you can use them as a template for your own team's submission.

### Preparing a team's system

To prepare a team's system, call:

```
./prepare_team_wamv.bash example_team

# For your team you will run:
# ./prepare_team_wamv.bash <your_team_name>
```

This will call `generate_wamv.launch` from the files in `team_config/example_team` and store the generated files in `team_generated/example_team`.

### Preparing trials for a task

In this README, we will be using some vocabulary that will be clearly defined here.

* `task`: One of the major competition tasks. Eg. `station_keeping`.

* `trial`: A specific task with a specific set of environmental conditions (e.g., sea state, wind magnitude and direction, lighting, etc.). Each task has multiple trials. Each trial will have a specific world file associated with it.

* `trial_number`: Each task will have `n` trials. Each trial will be labelled with a trial\_number, which ranges from `0` to `n-1` inclusive.

To prepare all of the trials for a task, call:

```
./prepare_task_trials.bash example_task

#./prepare_task_trials.bash <task_name>
```

This will call `generate_worlds.launch` from `task_config/example_task.yaml` and store the generated files in `task_generated/example_task`.

Please note that we will be writing our own private .yaml files for the tasks. Essentially, the only difference between testing out your system with these steps and the real competition is that for the real competition, we will be creating our own `.yaml` files for tasks that you have not seen, which will vary the environmental conditions. We will not be trying to surprise you with the conditions, but we want to simply reward teams that are robust to different environmental conditions.

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

To keep the terminal output clean, all of the output will be stored in `multi_scripts/prepare_output/`. These convenience scripts are more bug-prone, so if you notice any issues, please submit an issue [here](https://bitbucket.org/osrf/vrx-docker/issues?status=new&status=open).

## Running trials 

In order to run a trial with a specific team, the prepare scripts above must have been called on the associated task and team before running.

### Running a single trial for a single team

To run a single trial with a specific team (in this case the team from`team_config/example_team` and the trial with trial\_number 0 associated with `task_config/example_task.yaml`), call:

```
./run_trial.bash example_team example_task 0

# For your team you will run:
# ./run_trial.bash <your_team_name> <task_name> <trial_number>
```

This will instantiate two Docker containers.

1. The simulation server container, which runs the VRX Gazebo simulation with the desired team WAM-V and desired task trial world.

2. The VRX team's container, which runs their system from the Dockerhub image in `team_config/<your_team_name>/dockerhub_image.txt`.

TODO(tylerlum): Describe ending sequence, logs, expected errors/warnings

### Running all trials for a given task for a single team

To run all trials for a given task, call:

```
./multi_scripts/run_one_team_one_task.bash example_team example_task

# For your team you will run:
# ./multi_scripts/run_one_team_one_task.bash <your_team_name> <task_name>
```

This will run each of the trials for a given task sequentially in an automated fashion for one team.

### Running all trials for all tasks for a single team

_Only one trial config file is provided at the moment; this command will be more useful in the future._

To run all trials for all tasks listed in the `task_config` directory, call:

```
./multi_scripts/run_one_team_all_tasks.bash example_team

# For your team you will run:
# ./multi_scripts/run_one_team_all_tasks.bash <your_team_name>
```

This will run each of the trials for all tasks sequentially in an automated fashion. This is the invocation that will be used to test submissions for the Finals: your system will not be provided with any information about the conditions of the trials. If your system performs correctly with this invocation, regardless of the set of configuration files in the trial\_config directory, you're ready for the competition.

Note: To keep the terminal output clean, all of the output from multi_scripts will be stored in `multi_scripts/run_output/`. These convenience scripts are more bug-prone, so if you notice any issues, please submit an issue [here](https://bitbucket.org/osrf/vrx-docker/issues?status=new&status=open).

# Reviewing the results of a trial

## Reviewing the trial performance

After the trial has finished, you can go to `logs/<data_and_time>/<your_team_name>/<task_name>/<trial_number>/` to review the generated log files. TODO(tylerlum) Describe how to view performance specifically and show example. TODO(tylerlum) describe video directory and playback and logging

The `logs` directory has the following structure:

```
logs
│   ├── 2019-07-23.12-50-09_logs
│   │   └── example_team
│   │       └── example_task
│   │           └── 0
│   │               ├── gazebo-server
│   │               │   ├── log
│   │               │   │   └── 2019-07-23T125024.840259
│   │               │   │       └── gzserver
│   │               │   │           └── state.log
│   │               │   ├── ogre.log
│   │               │   └── server-11345
│   │               │       ├── default.log
│   │               │       └── gzserver.log
│   │               ├── ros-competitor
│   │               │   ├── rostopic_29_1563911438965.log
│   │               │   └── rostopic_30_1563911438980.log
│   │               ├── ros-server
│   │               │   ├── 18447f58-ad83-11e9-95f2-0242ac100016
│   │               │   │   ├── master.log
│   │               │   │   ├── roslaunch-d43eea66721e-50.log
│   │               │   │   ├── rosout-1-stdout.log
│   │               │   │   ├── rosout.log
│   │               │   │   ├── spawn_model-3.log
│   │               │   │   └── spawn_model-3-stdout.log
│   │               │   └── latest -> /home/tylerlum/.ros/log/18447f58-ad83-11e9-95f2-0242ac100016
│   │               ├── ros-server-latest
│   │               │   ├── master.log
│   │               │   ├── roslaunch-d43eea66721e-50.log
│   │               │   ├── rosout-1-stdout.log
│   │               │   ├── rosout.log
│   │               │   ├── spawn_model-3.log
│   │               │   └── spawn_model-3-stdout.log
│   │               └── vrx_rostopics.bag
│   ├── 2019-07-23.12-58-32_logs

```

The `logs` directory will have numerous directories with the date and time of creation. In each of those directories are the log files of each trial.

* gazebo-server/logs - contains the playback log file, which is used to visualize what happened and is used for video generation

* gazebo-server/server-xxxxx - contains the gazebo server logs

* ros-competitor - the log files from the competitor's container. Note: this is more prone to error, as finding the files depends on the competitor images's file structure

* ros-server - contains the log files from the ros server

## Playing back the simulation

To play back a specific trial's log file, move to `vrx-docker` and call:

```
roslaunch vrx_gazebo playback.launch log_file:=`pwd`/logs/<date_and_time>_logs/<your_team_name>/<task_name>/<trial_number>/gazebo-server/log/<data_and_time>/gzserver/state.log
```

## Development tips

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

## Files Required From VRX Teams For Submission

All VRX teams must submit one folder containing three files for automated evaluation. The name of the folder should be the name of the team. Please note that the filenames must be identical with how they are listed below.

1. `sensor_config.yaml`: The team's sensor configuration yaml file. One sensor configuration is used for all trials. For more information about this file, please refer to the [Creating a Custom WAM-V](https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20custom%20WAM-V%20Thruster%20and%20Sensor%20Configuration%20For%20Competition) tutorial.

2. `thruster_config.yaml`: The team's thruster configuration yaml file. One thruster configuration is used for all trials. For more information about this file, please refer to the [Creating a Custom WAM-V](https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20custom%20WAM-V%20Thruster%20and%20Sensor%20Configuration%20For%20Competition) tutorial.

3. `dockerhub_image.txt`: A text file containing only the name of their docker image publicly available on Dockerhub. Eg. `tylerlum/vrx-competitor-example:v2.2019`. For more information about this file, please refer to the [Creating a Dockerhub image for submission](https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20Dockerhub%20image%20for%20submission)

## Testing Your Submission

All teams should test their submissions by following the instructions above. It details how to run the scripts to test your system in a mock competition.

It is imperative that teams use the outlined process for testing their system, as it replicates the process that will be used during automated evaluation. If your system does not work in the mock competition setup, then it will not work for the real competition.

## Uploading Your Submission

Details about the submission will be coming shortly.
