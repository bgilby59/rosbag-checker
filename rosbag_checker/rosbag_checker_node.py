#!/usr/bin/env python3

import yaml
import rosbag2_py
import re
import rclpy
from rclpy.node import Node
import sys
# from rcl_interfaces.msg import ParameterDescriptor

class color: # ANSI codes for printing colored text
    GREEN = '\033[1;32;48m'
    YELLOW = '\033[1;33;48m'
    RED = '\033[1;31;48m'
    END = '\033[1;37;0m'

class RosbagCheckerNode(Node):
    def __init__(self):
        super().__init__("rosbag_checker_node")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bag_file', rclpy.Parameter.Type.STRING),
                ("topic_list", rclpy.Parameter.Type.STRING),
                ("topics", rclpy.Parameter.Type.STRING),
                ('check_frequency', True),
                ('default_frequency_requirements', [-1.0, sys.float_info.max])
            ]
        )

        try:
            self.bag = self.get_parameter('bag_file').get_parameter_value().string_value
        except:
            self.get_logger().error("Please set parameter bag_file to path of rosbag file")
        try:
            self.topic_list = self.get_parameter('topic_list').get_parameter_value().string_value
            self.use_yaml = True
            self.get_logger().info("Using input yaml file as topic list")
        except:
            self.use_yaml = False
            self.get_logger().warn("No input yaml file specified")
            try:
                self.topic_re = self.get_parameter('topics').get_parameter_value().string_value
                self.get_logger().info("Checking the topic name or the topics that match the regular expression {}".format(self.topic_re))
                # self.check_frequency = False
            except: 
                self.get_logger().error("Please give input yaml file or specify topics to check manually")

        self.check_frequency = self.get_parameter('check_frequency').get_parameter_value().bool_value
        if self.check_frequency:
            self.get_logger().info("Including check for frequency requirements")
            
        self.frequency_requirements = self.get_parameter('default_frequency_requirements').get_parameter_value().double_array_value

        self.check_bag()


    def check_bag(self):
        topics_to_rate = {}
        if self.use_yaml: # 1a. Get list of topics to check and their rate requirements from yaml file
            yaml_file_loc = self.topic_list

            with open(yaml_file_loc, 'r') as file:
                topic_list = yaml.safe_load(file)

            for topic in topic_list['topics']:
                if self.check_frequency: 
                    try: # Get hz_range is specified in yaml file, else, use default frequency requirements
                        topics_to_rate[topic['name']] = (topic['hz_range'][0], topic['hz_range'][1])
                    except:
                        self.get_logger().warn("Could not find frequency requirements for topic: {}. Using default frequency requirements".format(topic['name']))
                        topics_to_rate[topic['name']] = (self.frequency_requirements[0], self.frequency_requirements[1])
                else:
                    topics_to_rate[topic['name']] = ()
        else: # 1b. Get regex cli parameter
            topics_to_rate[self.topic_re] = (self.frequency_requirements[0], self.frequency_requirements[1])


        # 2. Read rosbag and get duration of rosbag
        bag_info = self.read_rosbag()
        duration = bag_info.duration
        duration = duration.days*86400 + duration.seconds + duration.microseconds/1000000


        # 3. Loop through topic data and gather output string
        output_string = ''
        for topic in topics_to_rate:
            found_match = False
            for topic_info in bag_info.topics_with_message_count:
                if re.fullmatch(topic, topic_info.topic_metadata.name): # Print information from all topics in rosbag that match topic name or regex we want to monitor
                    found_match = True
                    color_to_use = color.GREEN
                    msg_count = topic_info.message_count
                    msg_rate = topic_info.message_count/duration

                    if msg_count == 0:
                        color_to_use = color.RED
                    elif self.check_frequency:
                        min_rate, max_rate = topics_to_rate[topic]
                        if (msg_rate < min_rate or msg_rate > max_rate):
                            color_to_use = color.YELLOW

                    output_string += "{}Statistics for topic {}{}\n".format(color_to_use, topic_info.topic_metadata.name, color.END)
                    output_string += "{}Message count = {}, Message frequency = {}{}".format(color_to_use, msg_count, msg_rate, color.END)
                    output_string += "\n" + "\n"

            if not found_match: # If topic name or regex finds NO matches, then output information about that
                color_to_use = color.RED
                output_string += "{}Statistics for topic {}{}\n".format(color_to_use, topic, color.END)
                output_string += "{}Message count = {}, Message frequency = {}{}".format(color_to_use, 0, 0, color.END)
                output_string += "\n" + "\n"


        # 4. Output final results
        self.get_logger().info("Results: \n" + output_string)
    

    def read_rosbag(self):
        bag_info_obj = rosbag2_py.Info()

        if self.bag.endswith('.db3'):
            bag_info = bag_info_obj.read_metadata(self.bag, 'sqlite3') 
        elif self.bag.endswith('.mcap'):
            bag_info = bag_info_obj.read_metadata(self.bag, 'mcap') # TODO: test this
        else:
            self.get_logger().error("Provided rosbag is not an sqlite3 file or an mcap file")
        
        return bag_info


def main(args=None):
    rclpy.init(args=args)
    node = RosbagCheckerNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()