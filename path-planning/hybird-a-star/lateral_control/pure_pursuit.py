"""
Path Tracking Control using Pure Pursuit
"""

import numpy as np

from lib.highway_env.highway_env.envs import LateralControlRacetrackEnv


class PurePursuit:
    def __init__(self, env: LateralControlRacetrackEnv, gain: float, min_distance: float):
        self.wheelbase = 5
        self.rear_length = 2.5

        self.env: LateralControlRacetrackEnv = env
        self.gain = gain
        self.min_distance = min_distance

        self.rear_wheel_position: np.array = np.array([])
        self.yaw: float = 0.0
        self.v: float = 0.0

        self.look_ahead_distance: float = 0.0
        self.look_ahead_position: np.array = np.array([])

    def update(self, obs):
        self._set_state(obs)
        self._calculate_look_ahead_position()

        position_difference = self.look_ahead_position - self.rear_wheel_position
        alpha = np.arctan2(position_difference[1], position_difference[0]) - self.yaw
        steering = np.arctan2(2.0 * self.wheelbase * np.sin(alpha), self.look_ahead_distance)

        return steering

    def _set_state(self, obs):
        self.yaw = obs[4]
        self.rear_wheel_position = np.array([
            obs[0] - self.rear_length * np.cos(self.yaw),
            obs[1] - self.rear_length * np.sin(self.yaw)
        ])
        self.v = np.sqrt(obs[2] ** 2 + obs[3] ** 2)

    def _calculate_look_ahead_position(self):
        self.look_ahead_distance = self.v * self.gain + self.min_distance
        self.look_ahead_position = self._find_closest_link_position()

    def _find_closest_link_position(self):
        lane_index = self.env.road.network.get_closest_lane_index(self.rear_wheel_position, self.yaw)
        lane = self.env.road.network.get_lane(lane_index)
        current_longitudinal, _ = lane.local_coordinates(self.rear_wheel_position)

        target_longitudinal = current_longitudinal + self.look_ahead_distance

        while lane.length <= target_longitudinal:
            target_longitudinal = target_longitudinal - lane.length
            lane_index = self.env.get_next_lane_index(lane_index)
            lane = self.env.road.network.get_lane(lane_index)

        return lane.position(target_longitudinal, 0)
