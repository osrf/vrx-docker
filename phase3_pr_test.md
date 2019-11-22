# Testing VRX 2019 Phase 2 Pull Requests

## Initial setup

Assumes you have VRX setup in `~/vrx_ws`

```
cd ~/vrx_ws/src
hg clone ssh://hg@bitbucket.org/osrf/vrx-docker
# Directory for vrx-events forks/branches
mkdir ~/vrx_comp && cd ~/vrx_comp
```

## Assigning PR Review

When a new PR comes in, add a bitbucket comment (e.g., "Reviewing.") so that the rest of the team knows you are reviewing the PR and avoid duplication of effort.

## Checkout the PR from the forked repo

A couple ways:

Edit .hg/hgrc to change the url to the fork url.  Then pull and checkout the branch.

OR

```
cd ~/vrx_comp
rm -r vrx-events
# CHANGE THE URL BELOW
hg clone ssh://hg@bitbucket.org/ziyun99/vrx-events
# CHANGE THE BRANCH NAME BELOW
hg update 2019_phase3_team_bumblebee

```

## Copy over the team files

```
TEAM=bumblebee
PHASE=phase3_vrx_challenge
cp -R -v ~/vrx_comp/vrx-events/2019/${PHASE}/${TEAM}/ ~/vrx_ws/src/vrx-docker/team_config/
```

## Check Propulsion and Sensor Compliance
```
source ~/vrx_ws/devel/setup.bash
~/vrx_ws/src/vrx-docker/prepare_team_wamv.bash ${TEAM}
cat ~/vrx_ws/src/vrx-docker/generated/team_generated/${TEAM}/compliant.txt 
```

## Check Access to DockerHub Image

Peek at the file
```
cat  ~/vrx_comp/vrx-events/2019/${PHASE}/${TEAM}/dockerhub_image.txt
```

Start to pull the image
```
cat  ~/vrx_comp/vrx-events/2019/${PHASE}/${TEAM}/dockerhub_image.txt | xargs docker pull
```

If it is link, could check by 
Open link in a browser:
```
cat team_config/${TEAM}/dockerhub_image.txt | xargs chromium-browser 
```

# Report back.

If it works post something like

```
The yaml config files evaluate as "compliant" and the docker image is accessible.  The submission appears to be sufficient.

The next step is to actually start executing your soluiton (the docker container) with the configured setup and the VRX tasks and runs.  This will take a few days.

If there are any last minute changes, please submit a new PR before 2019, November 23, 23:59 PST.
```

Approve and merge.