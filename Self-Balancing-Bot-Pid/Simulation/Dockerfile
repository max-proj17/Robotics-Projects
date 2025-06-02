# Use the official ROS 2 Humble desktop image
FROM osrf/ros:humble-desktop

# Set non-interactive frontend for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install required tools and set up the new ROS GPG key
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y \
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
