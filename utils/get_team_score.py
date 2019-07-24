import os
import sys

# Get arguments
team_name = sys.argv[1]

# Get directory path
dir_path = os.path.dirname(os.path.realpath(__file__))
team_directory = dir_path + '/../logs/' + team_name

# Sum up task scores into team score
team_score = 0
subdirectories = [dI for dI in os.listdir(team_directory) if os.path.isdir(os.path.join(team_directory, dI))]
for subdirectory in subdirectories:
    # Get task score
    task_score_filename = team_directory + '/' + subdirectory + '/task_score.txt'
    task_score_file = open(task_score_filename, 'r')
    task_score = task_score_file.read()

    # Add it to task score
    team_score += float(task_score)

# Write team score to file
team_score_filename = team_directory + '/team_score.txt'
f = open(team_score_filename, 'w+')
f.write("{}".format(team_score))

print("Successfully recorded team score in {}".format(team_score_filename))
f.close()

