# get_trial_score.py: A Python script that gets the last message of a trial's rosbag file, 
#                     then records the final trial score in a text file
#
# eg. python get_trial_score.py <team_name> <task_name> <trial_number>

import os
import rosbag
import sys

# Get arguments
team_name = sys.argv[1]
task_name = sys.argv[2]
trial_number = sys.argv[3]

# Get directory path
dir_path = os.path.dirname(os.path.realpath(__file__))
trial_directory = dir_path + '/../generated/logs/' + team_name + '/' + task_name + '/' + trial_number

# Open ros bag
bag_name = trial_directory + '/vrx_rostopics.bag'
bag = rosbag.Bag(bag_name)

# Get last message
last_topic = None
last_msg = None
last_t = None

for topic, msg, t in bag.read_messages(topics=['/vrx/task/info']):
    last_topic = topic
    last_msg = msg
    last_t = t

# Write trial score to file
trial_score_name = trial_directory + "/trial_score.txt"
f = open(trial_score_name, 'w+')
f.write("{}".format(last_msg.score))

print("Successfully recorded trial score in {}".format(trial_score_name))
f.close()
bag.close()

