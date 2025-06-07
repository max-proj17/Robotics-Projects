import numpy as np

class Kalman1D:
    def __init__(self, dt, Q_angle, Q_rate, R_angle, R_rate):
        """
        dt:        time step
        Q_angle:   process‐noise variance for angle
        Q_rate:    process‐noise variance for rate
        R_angle:   measurement‐noise variance for angle
        R_rate:    measurement‐noise variance for rate
        """
        self.dt = dt

        # State vector [angle; rate], initialized to zero
        self.x = np.array([[0.0],
                           [0.0]])        

        # Covariance matrix (2×2), start with identity
        self.P = np.eye(2)

        # State transition matrix F (2×2)
        #   [ 1   dt ]
        #   [ 0    1 ]
        self.F = np.array([[1.0, dt],
                           [0.0, 1.0]])

        # Observation matrix H (2×2). We measure both angle and rate:
        #   [ 1   0 ]
        #   [ 0   1 ]
        self.H = np.eye(2)

        # Process‐noise covariance Q (2×2)
        #   [ Q_angle    0     ]
        #   [   0     Q_rate   ]
        self.Q = np.array([[Q_angle, 0.0],
                           [0.0,    Q_rate]])

        # Measurement‐noise covariance R (2×2)
        #   [ R_angle    0    ]
        #   [   0     R_rate  ]
        self.R = np.array([[R_angle, 0.0],
                           [0.0,    R_rate]])

    def predict(self):
        # Predict state forward one step
        self.x = self.F @ self.x
        # Predict covariance forward
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        """
        z must be a 2×1 vector: [measured_angle; measured_rate]
        """
        # Innovation (measurement residual)
        y = z - (self.H @ self.x)

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain (2×2)
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Updated state estimate
        self.x = self.x + (K @ y)

        # Updated covariance
        self.P = (np.eye(2) - K @ self.H) @ self.P

    def get_angle(self):
        return float(self.x[0, 0])

    def get_rate(self):
        return float(self.x[1, 0])
