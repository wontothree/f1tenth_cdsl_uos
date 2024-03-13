"""
Path Tracking Control using Model Predictive Control (MPC)

Reference:
PythonRobotics Model Predictive Speed and Steering Control (author: Atsushi Sakai (@Atsushi_twi))
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py
"""

import cvxpy as cp
import numpy as np

from lib.highway_env.highway_env.envs import LateralControlRacetrackEnv
from road.lane import CircularLane


class ModelPredictiveControl:
    def __init__(self, env: LateralControlRacetrackEnv, R, Rd, Q, steering_constraint, steering_rate_constraint,
                 time_horizon: int, dt: float):
        self.env: LateralControlRacetrackEnv = env
        self.dt = dt

        self.x_number = 5
        self.u_number = 1
        self.time_horizon = time_horizon

        self.R = R
        self.Rd = Rd
        self.Q = Q
        self.steering_constraint = steering_constraint
        self.steering_rate_constraint = steering_rate_constraint

        self.cg_position: np.array = np.array([])
        self.yaw: float = 0.0
        self.yaw_rate: float = 0.0
        self.v: float = 0.0
        self.vy: float = 0.0
        self.vx: float = 0.0

        self.previous_steering: float = 0.0

    def update(self, obs):
        self._set_state(obs)
        steering = self._update_mpc()
        self.previous_steering = steering
        return steering

    def _set_state(self, obs):
        self.yaw_rate = (obs[4] - self.yaw) / self.dt
        self.yaw = obs[4]
        self.cg_position = np.array([obs[0], obs[1]])

        self.v = np.sqrt(obs[2] ** 2 + obs[3] ** 2)
        v_direction = np.arctan2(obs[3], obs[2])
        self.vx = self.v * np.cos(v_direction - self.yaw)
        self.vy = self.v * np.sin(v_direction - self.yaw)

    def _update_mpc(self):
        x0 = self._get_state()

        x = cp.Variable((self.x_number, self.time_horizon + 1))
        u = cp.Variable((self.u_number, self.time_horizon))

        xref = np.zeros((self.x_number, self.time_horizon + 1))
        curvatures = self._calculate_reference_curvatures()

        cost = 0.0
        constraints = []
        for t in range(self.time_horizon - 1):
            cost += cp.quad_form(xref[:, t + 1] - x[:, t + 1], self.Q)
            cost += cp.quad_form(u[:, t], self.R)
            cost += cp.quad_form(u[:, t + 1] - u[:, t], self.Rd)

            A, B = self._get_dynamics_model(self.vx, curvatures[t])
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]
            constraints += [cp.abs(u[0, t + 1] - u[0, t]) <= self.steering_rate_constraint * self.dt]

        constraints += [x[:, 0] == x0]
        constraints += [cp.abs(u[0, 0] - self.previous_steering) <= self.steering_rate_constraint * self.dt]
        constraints += [cp.abs(u[0, :]) <= self.steering_constraint]

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.ECOS, verbose=False)

        if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
            steering_inputs = np.array(u.value[0, :]).flatten()

        else:
            print("Fail to find optimal solution.")
            steering_inputs = [0]

        steering = steering_inputs[0]
        return steering

    def _calculate_reference_curvatures(self):
        curvatures = [
            self._get_curvature(i * self.v * self.dt) for i in range(self.time_horizon + 1)
        ]
        return curvatures

    def _get_curvature(self, longitudinal_offset: float):
        lane_index = self.env.road.network.get_closest_lane_index(self.cg_position, self.yaw)
        lane = self.env.road.network.get_lane(lane_index)
        current_longitudinal, _ = lane.local_coordinates(self.cg_position)

        target_longitudinal = current_longitudinal + longitudinal_offset

        while lane.length <= target_longitudinal:
            target_longitudinal = target_longitudinal - lane.length
            lane_index = self.env.get_next_lane_index(lane_index)
            lane = self.env.road.network.get_lane(lane_index)

        if isinstance(lane, CircularLane):
            k = 1.0 / float(lane.radius)
        else:
            k = 0.0

        return k

    def _get_state(self):
        lane_index = self.env.road.network.get_closest_lane_index(self.cg_position, self.yaw)
        lane = self.env.road.network.get_lane(lane_index)
        long, lateral_offset = lane.local_coordinates(self.cg_position)
        heading_offset = lane.local_angle(self.yaw, long)
        return [self.vx, self.vy, self.yaw_rate, heading_offset, lateral_offset]

    def _get_dynamics_model(self, x_dot, k):
        # Highway Env의 Dynamics의 Parameter 값 참고
        Caf = 30000
        Car = 30000
        m = 1000
        lf = 2.5
        lr = 2.5
        Iz = 1 / 12 * m * (2 ** 2 + 5 ** 2)
        m = 1000

        A_c = np.array([
            [0, 0, 0, 0, 0],
            [0, -(Caf + Car) / (m * x_dot), -(Caf * lf - Car * lr) / (m * x_dot), 0, 0],
            [0, -(Caf * lf - Car * lr) / (Iz * x_dot), -(Caf * lf ** 2 + Car * lr ** 2) / (Iz * x_dot), 0, 0],
            [-k, 0, 1, 0, 0],
            [0, 1, 0, x_dot, 0],
        ])

        B_c = np.array([
            [0],
            [Caf / m],
            [Caf * lf / Iz],
            [0],
            [0]
        ])

        A = np.eye(self.x_number) + self.dt * A_c
        B = self.dt * B_c

        return A, B
