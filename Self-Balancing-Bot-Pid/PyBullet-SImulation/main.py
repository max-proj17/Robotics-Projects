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

# Add PID tuning sliders
p.addUserDebugParameter("P_gain", 0.0, 500.0, 200.0)
p.addUserDebugParameter("I_gain", 0.0, 100.0, 20.0)
p.addUserDebugParameter("D_gain", 0.0, 5.0, 0.075)

# Simulation config
dt = 1.0 / 240.0
noise_sigma = 0.02
max_force = 10.0
kf = Kalman1D(dt, 1e-5, 1e-3, 1e-2)
pid = PID(317.0, 10.0, 1.0)
data_queue = queue.Queue()
pitch_buffer = []  # For ESS calculation


def sim_thread():
    start_time = time.time()
    while True:
        t = time.time() - start_time
        _, orient = p.getLinkState(robotId, imu_idx)[:2]
        euler = p.getEulerFromQuaternion(orient)
        pitch = euler[0] + np.random.normal(0.0, noise_sigma)  # Roll axis = pitch

        # Update PID values from GUI
        pid.kp = p.readUserDebugParameter(0)
        pid.ki = p.readUserDebugParameter(1)
        pid.kd = p.readUserDebugParameter(2)

        # Kalman filter update
        kf.predict()
        kf.update(pitch)
        pitch_est = kf.get_angle()

        # Store pitch error for ESS (after stabilization time)
        pitch_buffer.append(pitch_est)
        if len(pitch_buffer) > int(1.0 / dt):  # ~last 1 sec
            pitch_buffer.pop(0)

        # PID control: desired angle = 0, so error = -pitch_est
        u = pid.compute(-pitch_est, dt)

        # Apply velocity commands
        p.setJointMotorControl2(robotId, left_wheel_jid, controlMode=p.VELOCITY_CONTROL, targetVelocity=u,
                                force=max_force)
        p.setJointMotorControl2(robotId, right_wheel_jid, controlMode=p.VELOCITY_CONTROL, targetVelocity=-u,
                                force=max_force)

        p.stepSimulation()
        time.sleep(dt)
        data_queue.put((t, pitch, pitch_est, u))


def plot_thread():
    plt.ion()
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(6, 7))
    line1, = ax1.plot([], [], 'r-', label='Noisy pitch')
    line2, = ax1.plot([], [], 'g-', label='Filtered pitch')
    ax1.set_ylim(-1, 1)
    ax1.set_xlim(0, 10)
    ax1.set_ylabel("Pitch (rad)")
    ax1.legend()

    line3, = ax2.plot([], [], 'b-', label='PID output')
    ax2.set_ylim(-30, 30)
    ax2.set_xlim(0, 10)
    ax2.set_ylabel("Control (velocity)")
    ax2.legend()

    line4, = ax3.plot([], [], 'm-', label='ESS')
    ax3.set_ylim(0, 0.5)
    ax3.set_xlim(0, 10)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("ESS (rad)")
    ax3.legend()

    t_data, raw_data, filt_data, ctrl_data, ess_data = [], [], [], [], []
    while True:
        while not data_queue.empty():
            t, r, f, c = data_queue.get()
            t_data.append(t)
            raw_data.append(r)
            filt_data.append(f)
            ctrl_data.append(c)

            ess = np.abs(np.mean(pitch_buffer)) if pitch_buffer else 0.0
            ess_data.append(ess)

        if t_data:
            # keep only last 10 seconds
            while t_data and (t_data[-1] - t_data[0] > 10):
                t_data.pop(0)
                raw_data.pop(0)
                filt_data.pop(0)
                ctrl_data.pop(0)
                ess_data.pop(0)

            line1.set_data(t_data, raw_data)
            line2.set_data(t_data, filt_data)
            line3.set_data(t_data, ctrl_data)
            line4.set_data(t_data, ess_data)
            ax1.set_xlim(max(0, t_data[-1] - 10), t_data[-1])
            ax2.set_xlim(max(0, t_data[-1] - 10), t_data[-1])
            ax3.set_xlim(max(0, t_data[-1] - 10), t_data[-1])
            fig.canvas.draw()
            fig.canvas.flush_events()
        time.sleep(0.01)


threading.Thread(target=sim_thread, daemon=True).start()
plot_thread()
