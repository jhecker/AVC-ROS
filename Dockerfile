# AVC-ROS Dockerfile

# Start from ROS’s Perception metapackage
FROM ros:kinetic-perception

# Update existing packages and install additional packages
# Note: This command produces the warning "debconf: delaying package configuration, 
#   since apt-utils is not installed", which is related to the fact that we are 
#   installing packages non-interactively. We can safely ignore it.
RUN apt-get update && apt-get install -y \
    python-catkin-tools \
    ros-kinetic-robot-localization

# Add standard swarmie user
RUN useradd -ms /bin/bash swarmie

# Add swarmie to dialout group to enable serial communication
RUN usermod -a -G dialout swarmie

# Set swarmie as default user
USER swarmie
WORKDIR /home/swarmie

# Set up ROS environment for user shell
RUN /bin/bash -c "echo 'source /opt/ros/kinetic/setup.bash' >> /home/swarmie/.bashrc"

# Pull AVC-ROS into container under swarmie user
RUN git clone https://github.com/jhecker/AVC-ROS.git

# Pull AVC-ROS submodules
RUN cd AVC-ROS && \
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
    catkin config --install ;\
    catkin build --no-status --no-summary --no-notify" >/dev/null 2>/dev/null

# Set up AVC-ROS environment for user shell
RUN /bin/bash -c "echo 'source /home/swarmie/AVC-ROS/install/setup.bash' >> /home/swarmie/.bashrc"
