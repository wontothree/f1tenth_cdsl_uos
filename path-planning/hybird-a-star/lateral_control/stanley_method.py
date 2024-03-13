"""
Path Tracking Control using Stanley Method
"""

import random

import numpy as np

from lib.highway_env.highway_env.envs import LateralControlRacetrackEnv


class StanleyMethod:
    def __init__(self, env: LateralControlRacetrackEnv, gain: float, has_noise=False):
        self.front_length = 2.5

        self.env: LateralControlRacetrackEnv = env
        self.gain = gain

        self.has_noise = has_noise

        self.front_wheel_position: np.array = np.array([])
        self.yaw: float = 0.0
        self.v: float = 0.0
        self.vy: float = 0.0
        self.vx: float = 0.0

    def update(self, obs):
        self._set_state(obs)
        lateral_offset, heading_offset = self._calculate_offset()
        if self.has_noise:
            heading_offset += random.gauss(0, 0.1)
        steering = heading_offset + np.arctan(self.gain * lateral_offset / self.vx)
        return steering

    def _set_state(self, obs):
        self.yaw = obs[4]
        self.front_wheel_position = np.array([
            obs[0] + self.front_length * np.cos(self.yaw),
            obs[1] + self.front_length * np.sin(self.yaw)
        ])
        self.v = np.sqrt(obs[2] ** 2 + obs[3] ** 2)
        v_direction = np.arctan2(obs[3], obs[2])
        self.vx = self.v * np.cos(v_direction - self.yaw)
        self.vy = self.v * np.sin(v_direction - self.yaw)

    def _calculate_offset(self):
        lane_index = self.env.road.network.get_closest_lane_index(self.front_wheel_position, self.yaw)
        lane = self.env.road.network.get_lane(lane_index)
        long, lateral_offset = lane.local_coordinates(self.front_wheel_position)
        heading_offset = lane.local_angle(self.yaw, long)
        return -lateral_offset, -heading_offset
