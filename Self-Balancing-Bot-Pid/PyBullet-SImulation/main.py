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
noise_sigma = 0.09
max_force = 10.0

# Now supplying both R_angle and R_rate to Kalman1D
kf = Kalman1D(dt,
              Q_angle=1e-5,
              Q_rate=1e-3,
              R_angle=1e-2,
              R_rate=1e-2)

pid = PID(317.0, 50.0, 1.0)
# pid = PID(317.0, 10.0, 1.0)
data_queue = queue.Queue()
pitch_buffer = []  # For ESS calculation

def sim_thread():
    start_time = time.time()
    while True:
        t = time.time() - start_time

        # Get orientation AND angular velocity from the IMU link
        link_state = p.getLinkState(robotId, imu_idx, computeLinkVelocity=1)
        # link_state returns a tuple; the last two elements are linear and angular velocity
        # Here we only need the angular velocity about the pitch axis (x‐axis)
        _, orient, _, _, _, _, _, ang_vel = link_state

        # Convert orientation to Euler; pitch = euler[0]
        euler = p.getEulerFromQuaternion(orient)
        pitch = euler[0] + np.random.normal(0.0, noise_sigma)

        # Direct gyroscope measurement (angular velocity about x‐axis)
        gyro_rate = ang_vel[0]

        # Update PID gains from GUI sliders
        pid.kp = p.readUserDebugParameter(0)
        pid.ki = p.readUserDebugParameter(1)
        pid.kd = p.readUserDebugParameter(2)

        # Kalman filter predict + update, now using both angle & rate
        kf.predict()

        # Form measurement vector z = [pitch; gyro_rate]
        z = np.array([[pitch],
                      [gyro_rate]])
        kf.update(z)

        # Extract filtered estimates
        pitch_est = kf.get_angle()
        rate_est  = kf.get_rate()

        # Store pitch for ESS (after stabilization)
        pitch_buffer.append(pitch_est)
        if len(pitch_buffer) > int(1.0 / dt):  # ~last 1 second
            pitch_buffer.pop(0)

        # PID control: desired angle = 0 → error = -pitch_est
        u = pid.compute(-pitch_est, dt)

        # Apply velocity commands to wheels (opposite sign on right wheel)
        p.setJointMotorControl2(robotId,
                                 left_wheel_jid,
                                 controlMode=p.VELOCITY_CONTROL,
                                 targetVelocity=u,
                                 force=max_force)
        p.setJointMotorControl2(robotId,
                                 right_wheel_jid,
                                 controlMode=p.VELOCITY_CONTROL,
                                 targetVelocity=-u,
                                 force=max_force)

        p.stepSimulation()
        time.sleep(dt)

        # Push data for plotting: (time, raw_pitch, filtered_pitch, control, raw_rate, filtered_rate)
        data_queue.put((t, pitch, pitch_est, gyro_rate, rate_est, u))

def plot_thread():
    plt.ion()
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(8, 6), sharex=True)

    # Noisy vs. Filtered Pitch
    line1_noisy, = ax1.plot([], [], 'r-', label='Noisy pitch')
    line1_filt,  = ax1.plot([], [], 'g-', label='Filtered pitch')
    ax1.set_ylabel("Pitch (rad)")
    ax1.set_ylim(-3, 3)  # Extended range for pitch
    ax1.legend()

    # Noisy vs. Filtered Rate
    line2_noisy, = ax2.plot([], [], 'b-', label='Gyro rate')
    line2_filt,  = ax2.plot([], [], 'c-', label='Filtered rate')
    ax2.set_ylabel("Angular rate (rad/s)")
    ax2.set_ylim(-10, 10)  # Longer y-axis for rate
    ax2.legend()

    # PID output (velocity command)
    line3, = ax3.plot([], [], 'm-', label='PID output')
    ax3.set_ylabel("Control (velocity)")
    ax3.set_ylim(-10, 10)  # Range from -20 to 20
    ax3.legend()

    # ESS (error sum over last second)
    line4, = ax4.plot([], [], 'k-', label='ESS')
    ax4.set_ylabel("ESS (rad)")
    ax4.set_xlabel("Time (s)")
    ax4.legend()

    t_data, raw_pitch, filt_pitch = [], [], []
    raw_rate, filt_rate, ctrl_data, ess_data = [], [], [], []

    while True:
        while not data_queue.empty():
            t, rp, fp, rr, fr, c = data_queue.get()
            t_data.append(t)
            raw_pitch.append(rp)
            filt_pitch.append(fp)
            raw_rate.append(rr)
            filt_rate.append(fr)
            ctrl_data.append(c)

            ess = abs(np.mean(pitch_buffer)) if pitch_buffer else 0.0
            ess_data.append(ess)

        if t_data:
            # keep only last 10 seconds
            while t_data and (t_data[-1] - t_data[0] > 10.0):
                t_data.pop(0)
                raw_pitch.pop(0)
                filt_pitch.pop(0)
                raw_rate.pop(0)
                filt_rate.pop(0)
                ctrl_data.pop(0)
                ess_data.pop(0)

            # Update plot lines
            line1_noisy.set_data(t_data, raw_pitch)
            line1_filt.set_data(t_data, filt_pitch)

            line2_noisy.set_data(t_data, raw_rate)
            line2_filt.set_data(t_data, filt_rate)

            line3.set_data(t_data, ctrl_data)
            line4.set_data(t_data, ess_data)

            for ax in (ax1, ax2, ax3, ax4):
                ax.set_xlim(max(0, t_data[-1] - 10.0), t_data[-1])

            fig.canvas.draw()
            fig.canvas.flush_events()

        time.sleep(0.01)

# Launch threads
threading.Thread(target=sim_thread, daemon=True).start()
plot_thread()
