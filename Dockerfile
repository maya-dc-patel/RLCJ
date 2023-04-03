# This dockerfile is used to build a ros-foxy image with Jackal packages.
# Use the official image as a parent image.
FROM ros:foxy

# Switch to bash and ensure system up to date.
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install git python3-pip -y

# Make workspace.
RUN mkdir -p /opt/ros2_ws/src
WORKDIR /opt/ros2_ws

# Clone Jackal packages and install dependencies.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && git clone -b foxy-devel https://github.com/jackal/jackal.git src/jackal \
    && git clone -b foxy-devel https://github.com/jackal/jackal_desktop.git src/jackal_desktop \
    && git clone -b foxy-devel https://github.com/jackal/jackal_simulator.git src/jackal_simulator \
    && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Install rlcj dependencies.
COPY requirements.txt src/rlcj/requirements.txt
RUN pip3 install -r src/rlcj/requirements.txt

# Override Jackal xacro and world sdf and build Jackal packages.
COPY jackal_custom.urdf.xacro src/jackal/jackal_description/urdf/jackal.urdf.xacro
COPY jackal_race_custom.world src/jackal_simulator/jackal_gazebo/worlds/jackal_race.world
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build \
    && source install/setup.sh

# Install rlcj.
COPY . src/rlcj
RUN pip3 install -e src/rlcj

ENTRYPOINT ["/opt/ros2_ws/src/rlcj/ros_entrypoint.sh"]
