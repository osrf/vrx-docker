# get_task_score.py: A Python script that sums up a task's trial scores from text files
#                     then records the final task score in a text file
#
# eg. python get_task_score.py <team_name> <task_name>

import os
import sys

# Get arguments
team_name = sys.argv[1]
task_name = sys.argv[2]

# Get directory path
dir_path = os.path.dirname(os.path.realpath(__file__))
task_directory = dir_path + '/../logs/' + team_name + '/' + task_name

# Sum up trial scores into task score
task_score = 0
subdirectories = [dI for dI in os.listdir(task_directory) if os.path.isdir(os.path.join(task_directory, dI))]
for subdirectory in subdirectories:
    # Get trial score
    trial_score_filename = task_directory + '/' + subdirectory + '/trial_score.txt'
    trial_score_file = open(trial_score_filename, 'r')
    trial_score = trial_score_file.read()

    # Add it to task score
    task_score += float(trial_score)

# Write task score to file
task_score_filename = task_directory + '/task_score.txt'
f = open(task_score_filename, 'w+')
f.write("{}".format(task_score))

print("Successfully recorded task score in {}".format(task_score_filename))
f.close()

