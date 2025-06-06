FROM osrf/ros:humble-desktop

# Set non-interactive frontend for apt
ENV DEBIAN_FRONTEND=noninteractive

# Remove the old ROS 2 apt list if it exists
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list

# Install necessary tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release && \
    rm -rf /var/lib/apt/lists/*

# Add the updated ROS 2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Update package lists and install necessary packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl vim build-essential \
    x11-apps mesa-utils glmark2 \
    libgl1-mesa-glx libgl1-mesa-dri \
    libx11-xcb1 libxcb1 libxext6 libxrender1 libxcb-icccm4 libxcb-image0 \
    libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-shape0 \
    libxcb-sync1 libxcb-xfixes0 libxcb-xinerama0 libxcb-glx0 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-effort-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-plugins \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS environment in shell
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create and set ROS workspace
RUN mkdir -p /home/dev/ros2_ws/src
WORKDIR /home/dev/ros2_ws

# Environment variables
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
