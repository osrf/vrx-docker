import os
import rosbag
import sys

# Open ros bag
bag_name = sys.argv[1]
print(bag_name)
bag = rosbag.Bag(bag_name)

# Get last message
last_topic = None
last_msg = None
last_t = None
# TODO: change to proper topic (before didn't have /vrx/task/info)
for topic, msg, t in bag.read_messages(topics=['/vrx/task/info']):
    last_topic = topic
    last_msg = msg
    last_t = t

# Write score to file
path = os.path.dirname(bag_name)
score_name = path + "/score.txt"
f = open(score_name, 'w+')
f.write("{}".format(msg.score))
f.close()
bag.close()

