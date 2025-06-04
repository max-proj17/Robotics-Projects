import pybullet as p
import pybullet_data
import time
import os
import math
import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import threading
import queue

# --- Kalman Filter and PID classes ---
class Kalman1D:
    def __init__(self, dt, Q_angle, Q_rate, R_angle):
        self.dt = dt
        self.x = np.array([[0.0], [0.0]])  # [angle, rate]
        self.P = np.eye(2)
        self.F = np.array([[1, dt], [0, 1]])
        self.H = np.array([[1, 0]])
        self.Q = np.array([[Q_angle, 0], [0, Q_rate]])
        self.R = np.array([[R_angle]])

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - (self.H @ self.x)                 # Innovation
        S = self.H @ self.P @ self.H.T + self.R   # Innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ self.H) @ self.P

    def get_angle(self):
        return self.x[0, 0]


class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


# --- PyBullet setup ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# Load a flat plane so wheels sit on z=0
planeId = p.loadURDF("plane.urdf")

# URDF path
urdf_path = os.path.join(os.path.dirname(__file__), "balance_bot.urdf")

# Place robot so wheels sit exactly on the plane (wheel_radius = 0.15)
wheel_radius = 0.15
chassis_height = 0.6
start_pos = [0, 0, (chassis_height / 2 + wheel_radius)]
# Tilt robot by 10° backward around the y-axis so controller must act right away
start_orientation = p.getQuaternionFromEuler([0, 0, 0])

robotId = p.loadURDF(
    urdf_path,
    start_pos,
    start_orientation,
    useFixedBase=False
)

# --- Discover all joints & apply default friction control ---
joint_map = {}
link_map = {}

num_joints = p.getNumJoints(robotId)
for j in range(num_joints):
    info = p.getJointInfo(robotId, j)
    joint_name = info[1].decode('utf-8')
    link_name = info[12].decode('utf-8')
    joint_map[joint_name] = j
    link_map[link_name] = j

    # Give every joint a small “static friction” via velocity control with zero target
    p.setJointMotorControl2(
        robotId,
        j,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0,
        force=0.05
    )

# Helper to find index by substring in link_name
def find_link_index(substring_list):
    for link_name, idx in link_map.items():
        lower = link_name.lower()
        if all(sub.lower() in lower for sub in substring_list):
            return idx
    return None

# Identify imu_link and wheel joints automatically
imu_link_index = find_link_index(["imu"])
left_wheel_jid = find_link_index(["left", "wheel"])   # joint index of left_wheel
right_wheel_jid = find_link_index(["right", "wheel"]) # joint index of right_wheel

print(f"Auto-discovered indices → imu_link_index: {imu_link_index}, "
      f"left_wheel_jid: {left_wheel_jid}, right_wheel_jid: {right_wheel_jid}")

# --- Simulation parameters ---
dt = 1.0 / 240.0
max_force = 10.0
noise_sigma = 0.02  # IMU noise standard deviation (radians)

# --- Kalman Filter & PID setup ---
kf = Kalman1D(dt, Q_angle=1e-5, Q_rate=1e-3, R_angle=1e-2)
pid = PID(Kp=20.0, Ki=1.0, Kd=1.0)  # Tuning these gains currently

# --- Queue for data transfer ---
data_queue = queue.Queue()

# Simulation thread function
def sim_thread():
    start_time = time.time()
    while True:
        now = time.time()
        t = now - start_time

        # 1) Read true pitch from IMU link
        link_state = p.getLinkState(robotId, imu_link_index)
        quat = link_state[1]  # world orientation quaternion of imu_link
        euler = p.getEulerFromQuaternion(quat)
        pitch_true = euler[1]  # pitch (rad)

        # 2) Add Gaussian noise
        noise = np.random.normal(0.0, noise_sigma)
        pitch_noisy = pitch_true + noise

        # 3) Kalman filter
        kf.predict()
        kf.update(pitch_noisy)
        pitch_est = kf.get_angle()

        # 4) PID control (aim to drive pitch_est → 0)
        error = -pitch_est
        u = pid.compute(error, dt)

        # 5) Command wheels (both same velocity to balance in place)
        p.setJointMotorControl2(
            robotId,
            left_wheel_jid,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=u,
            force=max_force
        )
        p.setJointMotorControl2(
            robotId,
            right_wheel_jid,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=u,
            force=max_force
        )

        # 6) Step simulation and sleep to maintain real time
        p.stepSimulation()
        time.sleep(dt)

        # 7) Put data into queue (non-blocking)
        data_queue.put((t, pitch_noisy, pitch_est, u))

# Plotting thread function
def plot_thread():
    plt.ion()
    fig, ax = plt.subplots()
    line1, = ax.plot([], [], 'r-', label='Noisy pitch (rad)')
    line2, = ax.plot([], [], 'g-', label='Filtered pitch (rad)')
    line3, = ax.plot([], [], 'b-', label='PID output')
    ax.set_ylim(-1.0, 1.0)        # pitch in ±1 rad range
    ax.set_xlim(0, 10)           # initial x-axis from 0 to 10 s
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Value")
    ax.legend(loc='upper right')

    t_data = []
    raw_data = []
    filt_data = []
    ctrl_data = []

    while True:
        # Retrieve all available data from queue
        while not data_queue.empty():
            t, raw, filt, ctrl = data_queue.get()
            t_data.append(t)
            raw_data.append(raw)
            filt_data.append(filt)
            ctrl_data.append(ctrl)

        if t_data:
            # Keep only last 10 seconds of data
            while t_data and (t_data[-1] - t_data[0] > 10.0):
                t_data.pop(0)
                raw_data.pop(0)
                filt_data.pop(0)
                ctrl_data.pop(0)

            # Update plot data
            line1.set_data(t_data, raw_data)
            line2.set_data(t_data, filt_data)
            line3.set_data(t_data, ctrl_data)

            # Scroll x-axis window
            ax.set_xlim(max(0, t_data[-1] - 10.0), t_data[-1])

            fig.canvas.draw()
            fig.canvas.flush_events()

        time.sleep(0.01)  # small pause to reduce CPU usage

# Start simulation in a background thread
threading.Thread(target=sim_thread, daemon=True).start()

# Run plotting loop in main thread
plot_thread()
