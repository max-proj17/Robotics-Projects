import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import threading
import queue
import time

from filters import Kalman1D
from controllers import PID
from simulation import setup_pybullet
import pybullet as p

# Setup PyBullet
robotId, imu_idx, left_wheel_jid, right_wheel_jid = setup_pybullet()
print(f"IMU: {imu_idx}, Left: {left_wheel_jid}, Right: {right_wheel_jid}")

# Simulation config
dt = 1.0 / 240.0
noise_sigma = 0.02
max_force = 10.0
kf = Kalman1D(dt, 1e-5, 1e-3, 1e-2)
pid = PID(40.0, 1.0, 1.5)
data_queue = queue.Queue()

def sim_thread():
    start_time = time.time()
    while True:
        t = time.time() - start_time
        euler = p.getEulerFromQuaternion(p.getLinkState(robotId, imu_idx)[1])
        pitch = euler[1] + np.random.normal(0.0, noise_sigma)
        kf.predict(); kf.update(pitch)
        pitch_est = kf.get_angle()
        u = pid.compute(-pitch_est, dt)

        for jid in [left_wheel_jid, right_wheel_jid]:
            p.setJointMotorControl2(robotId, jid, controlMode=p.VELOCITY_CONTROL, targetVelocity=u, force=max_force)

        p.stepSimulation(); time.sleep(dt)
        data_queue.put((t, pitch, pitch_est, u))

def plot_thread():
    plt.ion()
    fig, ax = plt.subplots()
    line1, = ax.plot([], [], 'r-', label='Noisy pitch')
    line2, = ax.plot([], [], 'g-', label='Filtered pitch')
    line3, = ax.plot([], [], 'b-', label='PID output')
    ax.set_ylim(-1, 1); ax.set_xlim(0, 10)
    ax.set_xlabel("Time"); ax.set_ylabel("Value"); ax.legend()

    t_data, raw_data, filt_data, ctrl_data = [], [], [], []

    while True:
        while not data_queue.empty():
            t, r, f, c = data_queue.get()
            t_data.append(t); raw_data.append(r); filt_data.append(f); ctrl_data.append(c)

        if t_data:
            while t_data and (t_data[-1] - t_data[0] > 10):
                t_data.pop(0); raw_data.pop(0); filt_data.pop(0); ctrl_data.pop(0)
            line1.set_data(t_data, raw_data)
            line2.set_data(t_data, filt_data)
            line3.set_data(t_data, ctrl_data)
            ax.set_xlim(max(0, t_data[-1] - 10), t_data[-1])
            fig.canvas.draw(); fig.canvas.flush_events()
        time.sleep(0.01)

threading.Thread(target=sim_thread, daemon=True).start()
plot_thread()