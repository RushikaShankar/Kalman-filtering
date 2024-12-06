# Kalman-filtering
# prompt: kalman filtering

import numpy as np

class KalmanFilter:
    def __init__(self, F, B, H, Q, R, P, x):
        self.F = F  # State transition matrix
        self.B = B  # Control input matrix
        self.H = H  # Observation matrix
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance
        self.P = P  # Estimate error covariance
        self.x = x  # State estimate

    def predict(self, u=0):
        # Predict the state and error covariance
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        # Update the state estimate and error covariance based on the measurement
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot(np.eye(len(self.x)) - np.dot(K, self.H), self.P)
        return self.x

# Example usage:
# Define the system matrices and parameters
F = np.array([[1, 1], [0, 1]])
B = np.array([[0.5], [0.1]])
H = np.array([[1, 0]])
Q = np.array([[0.1, 0], [0, 0.1]])
R = np.array([[1]])
P = np.array([[1, 0], [0, 1]])
x = np.array([[0], [0]])

# Create a Kalman filter object
kf = KalmanFilter(F, B, H, Q, R, P, x)

# Simulate measurements
measurements = [1, 2, 3, 4, 5]

# Perform Kalman filtering
for z in measurements:
  kf.predict()
  x_est = kf.update(np.array([[z]]))
  print("Estimated state:", x_est.flatten())
