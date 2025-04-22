# Use the official ROS 2 Humble base
FROM osrf/ros:humble-desktop

# Set non-interactive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary tools
RUN apt-get update && apt-get install -y \
    git \
    curl \
    vim \
    build-essential \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Source ROS setup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create ROS workspace
RUN mkdir -p /home/dev/ros2_ws/src
WORKDIR /home/dev/ros2_ws

# Set environment
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
