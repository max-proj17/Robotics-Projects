#!/bin/bash

source /opt/ros/humble/setup.bash

# Stop on error
set -e

# Go to workspace root (in case script is run from elsewhere)
cd "$(dirname "$0")"

echo "Building package..."
colcon build --packages-select balance_bot

echo "Sourcing workspace..."
source install/setup.bash

echo "Launching simulation..."
ros2 launch balance_bot balance_bot_gazebo_.launch.py