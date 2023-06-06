# Rosbag Checker

## Description
A ROS2 package to query the contents of a rosbag for message count and message frequency for given topic(s). 

### Input:
1. rosbag in `sqlite3` or `mcap` format.
2. one of the following:
    - `yaml` file containing a list of topics, and optionally, frequency requirements ([see example yaml file here for format](https://github.com/bgtier4/rosbag-checker/blob/python/input_yaml_format.yaml)).
    - single topic name or regular expression representing topics to be checked.

### Output:
The given list of topics with number of messages recorded by the rosbag, and message frequency.
- Green output indicates that messages corresponding to that topic were found and frequency requirements were met.
- Yellow output indicates that messages corresponding to that topic were found but frequency requirements were NOT met.
- Red output indicates that no messages were found corresponding to that topic.

## Requirements
- ROS2 Humble
- Python 3.10

## How to Install and Run
1. Download repository from github:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <REPO_URL>
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

**OR**

```bash
ros2 run rosbag_checker rosbag_checker --ros-args -p bag_file:=<PATH TO ROSBAG FILE> -p topics:=<TOPIC NAME OR REGEX>
```

## List of Parameters and Descriptions
- `bag_file`: path to rosbag file
- `topic_list`: path to yaml file containing lists of topics and optionally frequency requirements
- `topics`: name of topic or regular expression to check (alternative to topic_list)
- `check_frequency`: whether to check frequency requirements or not (`default: true`)
- `default_frequency_requirements`: default frequency requirements (`default: [-1, maximum float]`)
- `time_check_bag`: enable to run speed test on check bag function (`default: false`)
- `num_runs`: number of runs for speed test if speed test is enabled (`default: 1000`)
- `help`: show usage and list of parameters (`default: false`)