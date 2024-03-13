"""
Car Following Control using Linear Quadratic Regulator (LQR) with Delay (Practice)
"""

import numpy as np
import scipy


class LinearQuadraticRegulatorWithDelay:
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
        self.ego_vehicle_acceleration: float = 0.0
        self.ego_current_lane: int = 0
        self.front_vehicle: np.ndarray = np.array([])

        self.state_vector: np.array = np.array([[0.0], [0.0], [0.0]])

    def update(self, obs, ego_acceleration):
        self.update_ego_vehicle(obs, ego_acceleration)
        self.update_current_front_vehicle(obs)

        self.state_vector = self._get_state_vector()
        A, B = self._get_dynamics_model()
        K = self._calculate_regulator_gain(A, B)
        acceleration = -K * self.state_vector
        return acceleration.item()

    def update_ego_vehicle(self, obs, ego_acceleration):
        self.ego_vehicle = obs[0]
        self.ego_vehicle_acceleration = ego_acceleration
        self.ego_current_lane = self.find_current_lane(self.ego_vehicle)

    def update_current_front_vehicle(self, obs):
        # 전방 차량
        front_vehicles = [vehicle for vehicle in obs[1:] if vehicle[0] == 1 and vehicle[1] >= self.ego_vehicle[1]]

        current_front_vehicles = [vehicle for vehicle in front_vehicles if
                                  self.find_current_lane(vehicle) == self.ego_current_lane]
        self.front_vehicle = self.find_closest_vehicle(current_front_vehicles)

    def find_current_lane(self, vehicle) -> int:
        return int((vehicle[2] + self.lane_width / 2) // self.lane_width) + 1

    def find_closest_vehicle(self, vehicles: list) -> np.ndarray:
        if not vehicles:
            return np.array([])

        return min(vehicles, key=lambda vehicle: abs(vehicle[1] - self.ego_vehicle[1]))

    def _get_state_vector(self):
        relative_distance = self.front_vehicle[1] - self.ego_vehicle[1] - self.vehicle_length
        target_distance = self.ego_vehicle[3] * self.headway_time
        distance_error = target_distance - relative_distance
        velocity_error = self.front_vehicle[3] - self.ego_vehicle[3]

        return np.matrix([
            [distance_error],
            [velocity_error],
            [self.ego_vehicle_acceleration]
        ])

    def _get_dynamics_model(self):
        # TODO: Delay 를 고려한 Longitudinal Dynamics Model 구현

        A = None
        B = None

        return A, B

    def _calculate_regulator_gain(self, A, B):
        P = np.matrix(scipy.linalg.solve_discrete_are(A, B, self.Q, self.R))
        K = np.matrix(scipy.linalg.inv(B.T * P * B + self.R) * (B.T * P * A))
        return K
