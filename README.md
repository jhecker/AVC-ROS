AVC-ROS
==============

ROS code to control a Swarmie for the SparkFun AVC

## Prerequisites:
* Ubuntu 16.04 (Xenial Xerus)
* ROS Kinetic Kame

## Building:

```bash
# clone the AVC-ROS repository
git clone https://github.com/jhecker/AVC-ROS.git

# enter the main directory
cd AVC-ROS

# setup sweep-ros submodule
git submodule init
git submodule update

# build and install
catkin config --install
catkin build
```

## Running:

```bash
# enter main directory
cd /path/to/AVC-ROS

# source project
source install/setup.bash

# run main script
roslaunch launch/avc.launch
```
