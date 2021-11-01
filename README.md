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

In terminal 1 
```
roscore
```
Open another terminal 2
```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials talker
```

Open another terminal 3
```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials listener
```

