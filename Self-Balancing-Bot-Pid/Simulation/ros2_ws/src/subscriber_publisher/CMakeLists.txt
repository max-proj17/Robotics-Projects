cmake_minimum_required(VERSION 3.5)
project(subscriber_publisher)

# Use C++17 by default
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Executable: Simple Publisher Node
add_executable(publisher_node src/simple_publisher.cpp)
ament_target_dependencies(publisher_node rclcpp std_msgs)

# Executable: Simple Subscriber Node
add_executable(subscriber_node src/simple_subscriber.cpp)
ament_target_dependencies(subscriber_node rclcpp std_msgs)

# Install targets
install(TARGETS
  publisher_node 
  subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF/Xacro files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY gazebo
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
