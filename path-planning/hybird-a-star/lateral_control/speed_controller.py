"""
Speed Controller for Path Tracking Control
"""

import numpy as np

from lib.highway_env.highway_env.envs import LateralControlRacetrackEnv


class SpeedController:
    def __init__(self, env: LateralControlRacetrackEnv, gain: float = 1):
        self.env = env
        self.gain = gain

    def update(self, obs):
        self._set_state(obs)
        target_speed = self._get_target_speed()
        error = target_speed - self.speed

        return self.gain * error

    def _set_state(self, obs):
        self.heading = obs[4]
        self.position = np.array([obs[0], obs[1]])
        self.speed = np.sqrt(obs[2] ** 2 + obs[3] ** 2)

    def _get_target_speed(self) -> float:
        lane_index = self.env.road.network.get_closest_lane_index(self.position, self.heading)
        lane = self.env.road.network.get_lane(lane_index)
        return lane.speed_limit
