import numpy as np

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
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ self.H) @ self.P

    def get_angle(self):
        return self.x[0, 0]