import pybullet as p
import pybullet_data
import os
import math
import time
import numpy as np

def setup_pybullet():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    p.loadURDF("plane.urdf")

    urdf_path = os.path.join(os.path.dirname(__file__), "balance_bot.urdf")
    wheel_radius = 0.15
    chassis_height = 0.6
    start_pos = [0, 0, (chassis_height / 2 + wheel_radius)]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])

    robot_id = p.loadURDF(urdf_path, start_pos, start_orientation, useFixedBase=False)

    joint_map, link_map = {}, {}
    for j in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, j)
        joint_map[info[1].decode('utf-8')] = j
        link_map[info[12].decode('utf-8')] = j
        # Disable default motor forces
        p.setJointMotorControl2(robot_id, j, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0.05)

    def find_link_index(substrings):
        for name, idx in link_map.items():
            if all(sub.lower() in name.lower() for sub in substrings):
                return idx
        return None

    imu_index = find_link_index(["imu"])
    left_wheel = find_link_index(["left", "wheel"])
    right_wheel = find_link_index(["right", "wheel"])

    print(f"Auto-discovered indices → imu_link_index: {imu_index}, "
          f"left_wheel_jid: {left_wheel}, right_wheel_jid: {right_wheel}")

    # Increase lateral friction on both wheels
    for wheel_link_idx in (left_wheel, right_wheel):
        p.changeDynamics(robot_id, wheel_link_idx, lateralFriction=5.0)

    return robot_id, imu_index, left_wheel, right_wheel