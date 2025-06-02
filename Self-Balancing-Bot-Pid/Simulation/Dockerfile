# Use the official ROS 2 Humble desktop image
FROM osrf/ros:humble-desktop

# Set non-interactive frontend for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install tools, Gazebo packages, and GPU/graphics dependencies
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    # Refresh expired ROS key
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654 && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        git curl vim build-essential \
        x11-apps mesa-utils glmark2 vulkan-utils \
        libgl1-mesa-glx libgl1-mesa-dri \
        libx11-xcb1 libxcb1 libxext6 libxrender1 \
        libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
        libxcb-randr0 libxcb-render-util0 libxcb-shape0 \
        libxcb-sync1 libxcb-xfixes0 libxcb-xinerama0 libxcb-glx0 \
        nvidia-cuda-toolkit \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-joint-state-publisher-gui \
        ros-humble-xacro \
        ros-humble-ros2-control ros-humble-ros2-controllers \
        ros-humble-effort-controllers ros-humble-gazebo-ros2-control \
        ros-humble-gazebo-plugins && \
    rm -rf /var/lib/apt/lists/*

# Set up ROS environment in shell
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create and set ROS workspace
RUN mkdir -p /home/dev/ros2_ws/src
WORKDIR /home/dev/ros2_ws

# Environment variables
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
