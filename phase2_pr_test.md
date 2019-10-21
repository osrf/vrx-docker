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

When a new PR comes in, add a bitbucket comment so that the rest of the team knows you are reviewing the PR and avoid duplication of effort.

## Checkout the PR from the forked repo

E.g.,
```
rm -r vrx-events
hg clone ssh://hg@bitbucket.org/808brick/vrx-events
```

Copy over the team files
```
cd ~/vrx_ws/src/vrx-docker
cp -R ~/vrx_comp/vrx-events/2019/phase2_dress_rehearsal/Team_Kanaloa/ ./team_config/
```

## Check Propulsion and Sensor Compliance
```
source ~/vrx_ws/devel/setup.bash
./prepare_team_wamv.bash Team_Kanaloa
cat generated/team_generated/Team_Kanaloa/compliant.txt 
```

## Check Acess to DockerHub Image

Open link in a browser:
```
cat team_config/Team_Kanaloa/dockerhub_image.txt | xargs chromium-browser 
```

Can also begin to pull the image - I do this to confirm access, but terminate the download b/c I'm impatient.
```
docker pull 808brick/vrx_phase2
