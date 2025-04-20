# Robotics-Projects
A collection of my robotics projects, ranging from motor drivers to full robotic systems. This is where I implement the theory I know into practical projects.

## Balance Bot: Sensor-Fused Self-Balancing Robot (IN PROGRESS)

- Developed in C++ on the ESP32-S3 with FreeRTOS for real-time multitasking.
- Implements Proportional, Integral, and Derivative (PID) control for optimal balance and stability control.
- Utilizes a Complementary Filter for sensor fusion between IMU and magnetic encoders for accurate angle estimation.
- Incorporates both hardware and software low-pass filtering to reduce sensor noise.
- Reads sensor data via I²C (IMU) and direct ADC (magnetic encoders) for flexible signal acquisition.
- Designed for embedded control applications with an emphasis on modularity and signal processing.
- Utilizing a ROS2/Gazebo Docker container to run simulations of the robot's control algorithms.

---
## High level steps to install my setup (I will be making this more detailed soon)
### 1. Install WSL 2 and Docker.

### 2. Create a ROS 2 Dockerfile to set up the environment.

### 3. Set up the VSCode devcontainer configuration (devcontainer.json).

### 4. Open the project in VSCode and reopen it in the container.

### 5. Develop your ROS 2 packages, build them, and run your ROS 2 nodes.

### 6. Your ros2_ws is shared between the host and the container, allowing seamless development.
