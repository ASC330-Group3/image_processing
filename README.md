# platform_nav

platform_nav is a ROS package for navigation robotic platform using an overhead camera.
The is part of university group project for the ACS330 module.

## Installation

```
cd catkin_ws/src
git clone https://github.com/ASC330-Group3/platform_nav.git
cd platform_nav/nodes
git clone https://github.com/ASC330-Group3/OpenCV
cd ~/catkin_ws
catkin_make
source devel/setup.bash

```

## Usage

In terminal 1 run:
```
roscore
```
In terminal 2 run:
```
roslaunch platform_nav move_base
