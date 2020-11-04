# Testing VRX 2019 Phase 2 Pull Requests

## Initial setup

Assumes you have VRX setup in `~/vrx_ws`

```
cd ~/vrx_ws/src
git clone https://github.com/osrf/vrx-docker.git
# Directory for vrx-events forks/branches
mkdir ~/vrx_comp && cd ~/vrx_comp
```

## Assigning PR Review

When a new PR comes in, add a GitHub comment (e.g., "Reviewing.") so that the rest of the team knows you are reviewing the PR and avoid duplication of effort.

## Checkout the PR from the forked repo

A couple ways:

Edit .hg/hgrc to change the url to the fork url.  Then pull and checkout the branch.

OR

```
cd ~/vrx_comp
rm -r vrx-events
git clone https://github.com/osrf/vrx-events.git
```

## Copy over the team files

```
TEAM=bumblebee
cd ~/vrx_ws/src/vrx-docker
cp -R ~/vrx_comp/vrx-events/2019/phase2_dress_rehearsal/${TEAM}/ ~/vrx_ws/src/vrx-docker/team_config/
```

## Check Propulsion and Sensor Compliance
```
source ~/vrx_ws/devel/setup.bash
./prepare_team_wamv.bash Team_Kanaloa
cat generated/team_generated/Team_Kanaloa/compliant.txt 
```

## Check Acess to DockerHub Image

### For full URL, open in browser

Open link in a browser:
```
cat team_config/Team_Kanaloa/dockerhub_image.txt | xargs chromium-browser 
```

### For container name, try to pull

```
cat team_config/Team_Kanaloa/dockerhub_image.txt | xargs docker pull
```


Can also begin to pull the image - I do this to confirm access, but terminate the download b/c I'm impatient.
```
docker pull 808brick/vrx_phase2
```

## Respond to PR and Merge

```
Compliance tests for sensor and propulsion configurations passed.  Successfully able to access Docker image.

Next step is to evaluate run the docker image in the VRX simulation with the tasks/trials over the next few days.
```

## Summarize the configs

```
cd ~/vrx_ws/src/vrx-events/
hg pull && hg update && hg checkout default
cd ~/vrx_comp/vrx-private/utils/
hg pull && hg update && hg checkout summary
python summarize_entries.py
```
