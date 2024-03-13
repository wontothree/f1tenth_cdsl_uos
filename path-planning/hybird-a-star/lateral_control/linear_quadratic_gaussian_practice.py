"""
Path Tracking Control using Linear Quadratic Gaussian (LQG) (Practice)
"""

import random

import numpy as np
import scipy

from lib.highway_env.highway_env.envs import LateralControlRacetrackEnv


class LinearQuadraticGaussian:
    def __init__(self, env: LateralControlRacetrackEnv, Q: np.array, R: np.array, W: np.array, V: np.array, dt: float):
        self.wheelbase = 5

        self.env: LateralControlRacetrackEnv = env

        self.Q = Q
        self.R = R

        self.W = W
        self.V = V

        self.dt = dt

        self.cg_position: np.array = np.array([])
        self.yaw: float = 0.0
        self.v: float = 0.0
        self.vy: float = 0.0
        self.vx: float = 0.0

        self.C = np.matrix([
            [1, 0, 0, 0],
            [0, 0, 1, 0]
        ])

        self.measurement_vector = np.matrix([[0.0], [0.0]])
        self.state_vector_hat: np.array = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.steering = np.array([[0.0]])

        self.S = np.matrix([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ])

    def update(self, obs):
        self._set_state(obs)
        self.measurement_vector = self._get_measurement_vector()

        A, B = self._get_dynamics_model()

        # Kalman Filter
        L = self._calculate_observer_gain(A)
        # TODO (2): Kalman Filter 구현
        self.state_vector_hat = None
        # LQR
        K = self._calculate_regulator_gain(A, B)
        # TODO (3) : LQG Control Law 구현
        self.steering =0.0

        return self.steering.item()

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

        A_c = np.matrix([
            [0, 1, 0, 0],
            [0, -(Caf + Car) / m / self.vx, (Caf + Car) / m, (-Caf * lf + Car * lr) / m / self.vx],
            [0, 0, 0, 1],
            [0, -(Caf * lf - Car * lr) / Iz / self.vx, (Caf * lf - Car * lr) / Iz, -(Caf * lf * lf + Car * lr * lr) / Iz / self.vx]
        ])

        B_c = np.matrix([
            [0],
            [Caf / m],
            [0],
            [Caf * lf / Iz]
        ])

        A = np.eye(4) + self.dt * A_c
        B = self.dt * B_c
        return A, B

    def _calculate_observer_gain(self, A):
        # TODO (1): Observer Gain 계산
        L = None
        return L

    def _calculate_regulator_gain(self, A, B):
        P = np.matrix(scipy.linalg.solve_discrete_are(A, B, self.Q, self.R))
        K = np.matrix(scipy.linalg.inv(B.T * P * B + self.R) * (B.T * P * A))
        return K

    def _get_measurement_vector(self):
        lane_index = self.env.road.network.get_closest_lane_index(self.cg_position, self.yaw)
        lane = self.env.road.network.get_lane(lane_index)
        long, lateral_offset = lane.local_coordinates(self.cg_position)
        heading_offset = lane.local_angle(self.yaw, long)
        # TODO (4): Noise 가 추가된 LQG 활용
        return np.matrix([
            [lateral_offset],
            [heading_offset],
        ])
