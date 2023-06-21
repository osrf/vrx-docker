# Minimal Working Examples
This directory contains Dockerfiles for building example competitor Docker images.

## 2019

### `vrx_2019_simple`
* The minimal competitor image used to test scoring infrastructure in 2019.
* Corresponds to `example_team_2019` example in `team_config`.
* Note this is an approximate match to `tylerlum/vrx-competitor-example:v5.2019`

## 2022

### `vrx_2022_simple`
* The minimal competitor image used to test scoring infrastructure in 2019.
* Corresponds to `example_team` example in `team_config`.
* Note this is an approximate match to `crvogt/vrx_2022_simple`

### `vrx_2022_starter`
* Provides a starting point for competitors who expect to build a workspace using custom source code. 
* Because the Dockerfile expects to copy source code from the local file system, it is not self-contained.
* Please see the `2022/vrx_2022_starter/README.md` for build instructions. 
