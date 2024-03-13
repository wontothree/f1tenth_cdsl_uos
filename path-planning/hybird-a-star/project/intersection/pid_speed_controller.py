"""
Speed Control using PID
"""

import numpy as np


class PidSpeedController:
    def __init__(self, gain: float = 1):
        self.gain = gain

    def update(self, target_speed, obs):
        self._set_state(obs)
        error = target_speed - self.speed

        return self.gain * error

    def _set_state(self, obs):
        self.speed = np.linalg.norm(obs[3:5])
