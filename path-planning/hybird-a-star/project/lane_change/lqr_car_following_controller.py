"""
Car Following Control using Linear Quadratic Regulator (LQR)
"""

import numpy as np
import scipy


class LqrCarFollowingController:
    lane_width: float = 4.0
    vehicle_length: float = 5.0

    def __init__(self, headway_time, Q: np.array, R: np.array, dt):
        self.headway_time = headway_time
        self.dt = dt

        # Control Gain
        self.Q = Q
        # Regulator Gain
        self.R = R

        self.ego_vehicle: np.ndarray = np.array([])
        self.ego_current_lane: int = 0
        self.front_vehicle: np.ndarray = np.array([])

        self.state_vector: np.array = np.array([[0.0], [0.0]])

    def update(self, ego_vehicle, front_vehicle):
        self.ego_vehicle = ego_vehicle
        self.front_vehicle = front_vehicle

        self.state_vector = self._get_state_vector()
        A, B = self._get_dynamics_model()
        K = self._calculate_regulator_gain(A, B)
        acceleration = -K * self.state_vector
        return acceleration.item()

    def _get_state_vector(self):
        relative_distance = self.front_vehicle[1] - self.ego_vehicle[1] - self.vehicle_length
        target_distance = self.ego_vehicle[3] * self.headway_time
        distance_error = target_distance - relative_distance
        velocity_error = self.front_vehicle[3] - self.ego_vehicle[3]

        return np.matrix([
            [distance_error],
            [velocity_error],
        ])

    def _get_dynamics_model(self):
        A_c = np.matrix([
            [0, -1],
            [0, 0]
        ])

        B_c = np.matrix([
            [self.headway_time],
            [-1]
        ])

        A = np.eye(2) + self.dt * A_c
        B = self.dt * B_c

        return A, B

    def _calculate_regulator_gain(self, A, B):
        P = np.matrix(scipy.linalg.solve_discrete_are(A, B, self.Q, self.R))
        K = np.matrix(scipy.linalg.inv(B.T * P * B + self.R) * (B.T * P * A))
        return K
