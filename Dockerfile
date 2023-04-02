# This dockerfile is used to build a ros-foxy image with Jackal packages

# Use the official image as a parent image
FROM ros:foxy

# Source the setup.bash file
SHELL ["/bin/bash", "-c"] 

# Run the command inside your image filesystem
RUN mkdir /opt/ros2_ws

WORKDIR /opt/ros2_ws

# Create src folder

RUN mkdir src

# apt-get update

RUN apt-get update && apt-get install git -y

# Run Clone Jackal Packages

RUN git clone -b foxy-devel https://github.com/jackal/jackal.git /opt/ros2_ws/src/jackal && git clone -b foxy-devel https://github.com/jackal/jackal_desktop.git /opt/ros2_ws/src/jackal_desktop && git clone -b foxy-devel https://github.com/jackal/jackal_simulator.git /opt/ros2_ws/src/jackal_simulator

# rosdep install & source

RUN rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y && colcon build && source install/setup.bash

# ros entrypoint

ENTRYPOINT ["/ros_entrypoint.sh"]

