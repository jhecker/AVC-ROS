AVC-ROS
==============

ROS packages required to control a Swarmie for the SparkFun AVC

## Prerequisites:
* [Ubuntu 16.04 (Xenial Xerus)](http://releases.ubuntu.com/16.04/) (Desktop or Server)
* [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu) (either the perception [ROS metapackage](http://www.ros.org/reps/rep-0142.html), or a superset of this metapackage)
* [AVC-Arduino](https://github.com/jhecker/AVC-Arduino) running on a [Pololu A-Star](https://www.pololu.com/product/3104) microcontroller

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
