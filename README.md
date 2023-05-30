# Rosbag Checker

## Description
A ROS2 package to query the contents of a rosbag for message count and message frequency corresponding to a specific topic. 

Rosbag checker takes as input a rosbag in sqlite3 or mcap format and a yaml file with a list of topics, and optionally, frequency requirements (see example yaml file here for format).

For quick checking, the input yaml file can be replaced by a single topic name, or a regular expression representing the topics that should be checked.

## Requirements
- ROS2 Humble
- Python 3.10

## How to Install and Run
1. Download repository from github:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone REPO_URL_HERE
```

2. Build and source package

```bash
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

3. Run rosbag checker

```bash
ros2 run rosbag_checker rosbag_checker --ros-args -p bag_file:=<PATH TO ROSBAG FILE> -p topic_list:=<PATH TO INPUT YAML FILE>
```

## List of Parameters and Descriptions
- `bag_file`: path to rosbag file
- `topic_list`: path to yaml file containing lists of topics and optionally frequency requirements
- `topics`: name of topic or regular expression that should be checked
- `check_frequency`: whether to check frequency requirements or not (`default: true`)
- `default_frequency_requirements`: double array `[min_frequency, max_frequency]` representing default frequency requirements assigned to topics without specified frequency requirements (`default: [minimum float, maximum float]`)