AVC-ROS
==============

ROS packages required to control a Swarmie for the SparkFun AVC

## Initial machine configuration:

Add the following rules to ```/etc/udev/rules.d/99-usb-serial.rules```:
* ```SUBSYSTEM=="tty", ATTRS{idVendor}=="1ffb", ATTRS{idProduct}=="2300", SYMLINK+="swarmie/arduino"```
* ```SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a6", SYMLINK+="swarmie/ublox"```
* ```SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="swarmie/scanse"```

## Setup w/ Docker:

### Prerequisites:

* [Docker CE (Community Edition)](https://docs.docker.com/install/) installed on the OS of your choice

### Building:

```bash
# clone the AVC-ROS repository
git clone https://github.com/jhecker/AVC-ROS.git

# build the Docker image
docker build -t avc-ros AVC-ROS
```

### Running:

```bash
# derive and run the Docker container from the Docker image
docker run -it --rm --name avc-ros --hostname swarmie --privileged -v /dev/swarmie:/dev/swarmie --net=host --add-host swarmie:127.0.1.1 avc-ros

# run main script from inside the interactive Docker shell
roslaunch AVC-ROS/launch/avc.launch
```


## Setup w/o Docker:

### Prerequisites:

* [Ubuntu 16.04 (Xenial Xerus)](http://releases.ubuntu.com/16.04/) (Desktop or Server)
* [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu) (either the perception [ROS metapackage](http://www.ros.org/reps/rep-0142.html), or a superset of this metapackage)
* The following ROS packages (available through APT) in addition to the perception metapackage:
  * ```python-catkin-tools```
  * ```ros-kinetic-robot-localization```
  * ```ros-kinetic-gmapping```
  * ```ros-kinetic-pointcloud-to-laserscan```
  * ```ros-kinetic-urg-node```

### Building:

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

### Running:

```bash
# enter main directory
cd /path/to/AVC-ROS

# source project
source install/setup.bash

# run main script
roslaunch launch/avc.launch
```
