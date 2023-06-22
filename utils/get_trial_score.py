# get_trial_score.py: A Python script that gets the last message of a trial's ros2bag file, 
#                     then records the final trial score in a text file
#
# eg. python get_trial_score.py <team_name> <task_name> <trial_number>

import os
import rosbag2_py
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Get arguments
team_name = sys.argv[1]
task_name = sys.argv[2]
trial_number = sys.argv[3]

# Get directory path
dir_path = os.path.dirname(os.path.realpath(__file__))
trial_directory = dir_path + '/../generated/logs/' + team_name + '/' + task_name + '/' + trial_number

# Open ros bag
bag_path = trial_directory + '/vrx_rostopics.bag/vrx_rostopics.bag_0.db3'
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', 
                                                output_serialization_format='cdr')

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

topic_types = reader.get_all_topics_and_types()

type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

storage_filter = rosbag2_py.StorageFilter(topics=['/vrx/task/info'])
reader.set_filter(storage_filter)

# Get last message
last_topic = None
last_msg_data = None
last_t = None

while reader.has_next():
    (last_topic, last_msg_data, last_t) = reader.read_next() 

msg_type = get_message(type_map[last_topic])
last_msg = deserialize_message(last_msg_data, msg_type)
score = None
for field in last_msg.params:
  if field.name == "score":
    score = field.value.double_value
# Write trial score to file
trial_score_name = trial_directory + "/trial_score.txt"
with open(trial_score_name, 'w+') as f:
  f.write("{}".format(score))

print("Successfully recorded trial score in {}".format(trial_score_name))
