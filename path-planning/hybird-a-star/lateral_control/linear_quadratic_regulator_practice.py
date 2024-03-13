"""
Path Tracking Control using Linear Quadratic Regulator (LQR) (Practice)
"""

import random

import numpy as np
import scipy

from lib.highway_env.highway_env.envs import LateralControlRacetrackEnv


class LinearQuadraticRegulator:
    def __init__(self, env: LateralControlRacetrackEnv, Q: np.array, R: np.array, dt: float):
        self.wheelbase = 5

        self.env: LateralControlRacetrackEnv = env

        self.Q = Q
        self.R = R

        self.dt = dt

        self.cg_position: np.array = np.array([])
        self.yaw: float = 0.0
        self.v: float = 0.0
        self.vy: float = 0.0
        self.vx: float = 0.0

        self.state_vector: np.array = np.array([[0.0], [0.0], [0.0], [0.0]])

    def update(self, obs):
        self._set_state(obs)
        self.state_vector = self._get_state_vector()

        A, B = self._get_dynamics_model()

        # LQR
        K = self._calculate_regulator_gain(A, B)
        # TODO (3) : LQR Control Law 구현
        steering = 0.0
        return steering.item()

    def _set_state(self, obs):
        self.yaw = obs[4]
        self.cg_position = np.array([obs[0], obs[1]])
        self.v = np.sqrt(obs[2] ** 2 + obs[3] ** 2)
        v_direction = np.arctan2(obs[3], obs[2])
        self.vx = self.v * np.cos(v_direction - self.yaw)
        self.vy = self.v * np.sin(v_direction - self.yaw)

    def _get_dynamics_model(self):
        # Highway Env의 Dynamics의 Parameter 값 참고
        Caf = 30000
        Car = 30000
        m = 1000
        lf = 2.5
        lr = 2.5
        Iz = 1 / 12 * m * (2 ** 2 + 5 ** 2)

        # TODO (1) : A, B Matrix 계산
        A = None
        B = None
        return A, B

    def _calculate_regulator_gain(self, A, B):
        # TODO (2) : Regulator Gain 계산
        K = None
        return K

    def _get_state_vector(self):
        lane_index = self.env.road.network.get_closest_lane_index(self.cg_position, self.yaw)
        lane = self.env.road.network.get_lane(lane_index)
        long, lateral_offset = lane.local_coordinates(self.cg_position)
        heading_offset = lane.local_angle(self.yaw, long)

        return np.matrix([
            [lateral_offset],
            [(lateral_offset - self.state_vector[0].item()) / self.dt],
            [heading_offset],
            [(heading_offset - self.state_vector[2].item()) / self.dt]
        ])
