# AVC-ROS Dockerfile

# Start from ROS’s Perception metapackage
FROM ros:kinetic-perception

# Update existing packages and install additional packages
# Note: This command produces the warning "debconf: delaying package configuration, 
#   since apt-utils is not installed", which is related to the fact that we are 
#   installing packages non-interactively. We can safely ignore it.
RUN apt-get update && apt-get install -y \
    software-properties-common \
    python-catkin-tools \
    ros-kinetic-amcl \
    ros-kinetic-dwa-local-planner \
    ros-kinetic-follow-waypoints \
    ros-kinetic-gmapping \
    ros-kinetic-map-server \
    ros-kinetic-move-base \
    ros-kinetic-pointcloud-to-laserscan \
    ros-kinetic-robot-localization \
    ros-kinetic-teb-local-planner \
    ros-kinetic-urg-node

# Install ubuntu-make and the Arduino IDE for reprog'ing the microcontroller
RUN add-apt-repository ppa:lyzardking/ubuntu-make && \
    apt-get update && apt-get install -y ubuntu-make && \
    umake electronics arduino /usr/local/bin/arduino-ide && \
    ln -sfn /usr/local/bin/arduino-ide/arduino /usr/local/bin/arduino

# Add standard swarmie user
RUN useradd -ms /bin/bash swarmie

# Add swarmie to dialout group to enable serial communication
RUN usermod -a -G dialout swarmie

# Clone and install sweep-sdk
RUN git clone https://github.com/scanse/sweep-sdk.git && \
    cd sweep-sdk/libsweep && \
    mkdir build && \
    cd build && \
    cmake .. && \
    cmake --build . --target install

# Set library path to ensure that libsweep can be found by sweep-ros
ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/usr/local/lib

# Set swarmie as default user
USER swarmie
WORKDIR /home/swarmie

# Set up ROS environment for user shell
RUN /bin/bash -c "echo 'source /opt/ros/kinetic/setup.bash' >> /home/swarmie/.bashrc"

# Clone AVC-ROS into container under swarmie user
RUN git clone https://github.com/jhecker/AVC-ROS.git

# Clone AVC-ROS submodules
RUN cd AVC-ROS && \
    git submodule init && \
    git submodule update

# Clone AVC-Arduino into container
RUN git clone https://github.com/jhecker/AVC-Arduino.git

# Clone AVC-Arduino submodules
RUN cd AVC-Arduino && \
    git submodule init && \
    git submodule update

# Build and install AVC-ROS
# Note: We suppress all standard and error output here. Even the correct, expected
#   output from `catkin build` tends to look like errors to the user, so it is
#   preferable to mitigate confusion by silencing catkin's build status. One might
#   improve upon this hack in the future to allow true errors to be passed
#   through during the Docker container building process.
RUN /bin/bash -c "cd AVC-ROS ;\
    source /opt/ros/kinetic/setup.bash ;\
    catkin build --no-status --no-summary --no-notify" >/dev/null 2>/dev/null

# Set up AVC-ROS environment for user shell
RUN /bin/bash -c "echo 'source /home/swarmie/AVC-ROS/devel/setup.bash' >> /home/swarmie/.bashrc"
