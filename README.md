# VRX Automated Evaluation

This repository contains scripts and infrastructure that will be used for automatic evaluation of Virtual RobotX (VRX) teams' submissions. 

## Overview

This repository consists of two major components: 

1. The VRX server system, which runs the VRX Gazebo simulation

2. The VRX competitor system, which runs a team's control software.

For security and reproducibility, the VRX server and the VRX competitor's systems will be run in separate, isolated environments called Docker containers.

## Installing Docker

Docker is required to run the automated evaluation. Please follow the [Docker CE for Ubuntu tutorial's](https://docs.docker.com/install/linux/docker-ce/ubuntu) __Prerequisites__ and __Install Docker CE__ sections.

Then, continue to the [post-install instructions](https://docs.docker.com/engine/installation/linux/linux-postinstall/) and complete the __Manage Docker as a non-root user__ section to avoid having to run the commands on this page using `sudo`.


## Files Required From VRX Teams For Submission

All VRX teams must submit one folder containing three files for automated evaluation. The name of the folder should be the name of the team. Please note that the filenames must be identical with how they are listed below.

1. `sensor_config.yaml`: The team's sensor configuration yaml file. One sensor configuration is used for all trials. For more information about this file, please refer to the [Creating a Custom WAM-V](https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20custom%20WAM-V%20Thruster%20and%20Sensor%20Configuration%20For%20Competition) tutorial.

2. `thruster_config.yaml`: The team's thruster configuration yaml file. One thruster configuration is used for all trials. For more information about this file, please refer to the [Creating a Custom WAM-V](https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20custom%20WAM-V%20Thruster%20and%20Sensor%20Configuration%20For%20Competition) tutorial.

3. `dockerhub_url.txt`: A text file containing only the name of their docker image publicly available on Dockerhub. Eg. `vrx-competitor-example:v1.2019`. For more information about this file, please refer to the [Creating a Dockerhub image for submission](https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20Dockerhub%20image%20for%20submission)

## Testing Your Submission

All teams should test their submissions by following the instructions above. It details how to run the scripts to test your system in a mock competition.

It is imperative that teams use the outlined process for testing their system, as it replicates the process that will be used during automated evaluation. If your system does not work in the mock competition setup, then it will not work for the real competition.

## Uploading Your Submission

Details about the submission will be coming shortly.
