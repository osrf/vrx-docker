# get_team_score.py: A Python script that sums up a team's task scores
#                     then records the final team score in a text file
#
# eg. python get_team_score.py <team_name>

import os
import sys

# Get arguments
team_name = sys.argv[1]

# Get directory path
dir_path = os.path.dirname(os.path.realpath(__file__))
team_directory = dir_path + '/../generated/logs/' + team_name

# Sum up task scores into team score
team_score = []
subdirectories = [dI for dI in os.listdir(team_directory) if os.path.isdir(os.path.join(team_directory, dI))]
for subdirectory in subdirectories:
    # Get task score
    task_score_filename = team_directory + '/' + subdirectory + '/task_score.txt'
    task_score_file = open(task_score_filename, 'r')
    task_score = task_score_file.read()

    # Add it to team score
    team_score.append(task_score)

# Write team score to file
team_score_filename = team_directory + '/team_score.txt'
f = open(team_score_filename, 'w+')
for task_score in team_score:
    f.write(task_score + '\n')

print("Successfully recorded team score in {}".format(team_score_filename))
f.close()

