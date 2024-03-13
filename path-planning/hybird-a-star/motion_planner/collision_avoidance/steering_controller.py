"""
Steering Controller for Collision Avoidance Project
"""

import numpy as np


class SteeringController:
    def __init__(self, gain: float, min_distance: float):
        self.wheelbase = 5
        self.rear_length = 2.5

        self.gain = gain
        self.min_distance = min_distance

        self.rear_wheel_position: np.array = np.array([])
        self.yaw: float = 0.0
        self.v: float = 0.0

        self.look_ahead_distance: float = 0.0
        self.look_ahead_position: np.array = np.array([])

    def update(self, obs, xs, ys):
        self._set_state(obs)
        self._calculate_look_ahead_position(xs, ys)

        position_difference = self.look_ahead_position - self.rear_wheel_position
        alpha = np.arctan2(position_difference[1], position_difference[0]) - self.yaw
        delta = np.arctan2(2.0 * self.wheelbase * np.sin(alpha), self.look_ahead_distance)

        return delta

    def _set_state(self, obs):
        self.yaw = obs[5]
        self.rear_wheel_position = np.array([
            obs[1] - self.rear_length * np.cos(self.yaw),
            obs[2] - self.rear_length * np.sin(self.yaw)
        ])
        self.v = np.sqrt(obs[3] ** 2 + obs[4] ** 2)

    def _calculate_look_ahead_position(self, xs, ys):
        self.look_ahead_distance = self.v * self.gain + self.min_distance
        index = np.argmin(np.abs(xs - self.rear_wheel_position[0] - self.look_ahead_distance))
        self.look_ahead_position = np.array([xs[index], ys[index]])
