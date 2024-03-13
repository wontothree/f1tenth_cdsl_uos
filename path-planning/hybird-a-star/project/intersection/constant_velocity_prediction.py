"""
Constant Velocity Prediction Model
"""

import numpy as np


class ConstantVelocityPrediction:
    def __init__(self, prediction_horizon, dt):
        self.prediction_horizon = prediction_horizon
        self.dt = dt

    def predict(self, state):
        if not state.size:
            return np.array([])
        _, x, y, vx, vy, heading = state
        presence = 1 * np.ones(self.prediction_horizon + 1)
        x_pred = x + vx * np.arange(0, self.prediction_horizon + 1) * self.dt
        y_pred = y + vy * np.arange(0, self.prediction_horizon + 1) * self.dt
        vx_pred = vx * np.ones(self.prediction_horizon + 1)
        vy_pred = vy * np.ones(self.prediction_horizon + 1)
        heading_pred = heading * np.ones(self.prediction_horizon + 1)
        return np.stack([presence, x_pred, y_pred, vx_pred, vy_pred, heading_pred], axis=1)
