AVC-ROS
==============

ROS packages required to control a Swarmie for the SparkFun AVC

## Initial configuration:

If connecting to hardware ([Pololu A-Star microcontroller](https://www.pololu.com/product/3104), [MIKROE-1032 GPS board](https://www.mouser.com/ProductDetail/mikroElektronika/MIKROE-1032/?qs=sGAEpiMZZMuyGAGFEBEmZt%2fsR%2fWVGgy3), or [Scanse Sweep LiDAR](https://www.robotshop.com/en/sweep-v1-360-laser-scanner.html) (discontinued)), add the following rules to ```/etc/udev/rules.d/99-usb-serial.rules```:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1ffb", ATTRS{idProduct}=="2300", SYMLINK+="swarmie/arduino"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a6", SYMLINK+="swarmie/ublox"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="swarmie/scanse"
```

The [AVC-Arduino](https://github.com/jhecker/AVC-Arduino) repo provides an interface to access lower-level sensors ([Pololu AltIMU-10 v4](https://www.pololu.com/product/2470), [Pololu quadrature wheel encoders](https://www.pololu.com/product/2827), and [PING))) ultrasonic distance sensors](https://www.pololu.com/product/1605)), as well as [Pololu metal gearmotors](https://www.pololu.com/product/2827) through the [Pololu A-Star](https://www.pololu.com/product/3104).

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

  ```
  python-catkin-tools
  ros-kinetic-robot-localization
  ros-kinetic-gmapping
  ros-kinetic-pointcloud-to-laserscan
  ros-kinetic-urg-node
  ```
* [Sweep SDK](https://github.com/scanse/sweep-sdk) to provide API support to the Sweep ROS package

### Building:

```bash
# clone the AVC-ROS repository
git clone https://github.com/jhecker/AVC-ROS.git

# enter the main directory
cd AVC-ROS

# setup sweep-ros submodule
git submodule init
git submodule update

# build project
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
