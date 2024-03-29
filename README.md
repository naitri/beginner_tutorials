# beginner_tutorials


[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview
This repository contains beginner tutorials in C++ for a publisher and subscriber node in ROS for custom string message

## Dependencies
* ROS Noetic
* Ubuntu 20.04

## Build Instructions
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/naitri/beginner_tutorials
cd ..
catkin_make
```
## Run instructions
1. Go to directory
```
cd catkin_ws
```
2. Source the workspace
```
source devel/setup.bash
```
3. To run talker and listener nodes
In terminal 1 
```
roscore
```
Open another terminal 2
```
rosrun beginner_tutorials talker
```

Open another terminal 3
```
rosrun beginner_tutorials listener
```

4. To run talker and listener nodes using launch file syncronously where you can change rate of publishing
```
roslaunch beginner_tutorials beginner_tutorials.launch rate:=5
```

# Running ROS service
Use the follwing command to run ros service for changing the string
```
rosservice call /update "Homework 10"
```

# Recording rosbag using launch file

Use the following launch file to start recording topic for 15seconds
```
roslaunch beginner_tutorials beginner_tutorials.launch rate:=5 bag_record:=true
```
Verify the bag file:

```rosbag info <your bagfile>
```

Play the rosbag file
```
rosbag play record.bag
```

# Run recorded rosbag file

In one terminal start listener
```
source devel/setup.bash
rosrun beginner_tutorials listener
```
In another terminal play recorded bag file
```
cd results/
rosbag play record.bag
```

#Running tests
Build the tests
```
cd catkin_ws
catkin_make
catkin_make run tests
```

Run the tests
```
cd catkin_ws
rostest beginner_tutorials test_talker.test
```



