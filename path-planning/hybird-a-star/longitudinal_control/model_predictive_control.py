"""
Car Following Control using Model Predictive Control (MPC)

Reference:
PythonRobotics Model Predictive Speed and Steering Control (author: Atsushi Sakai (@Atsushi_twi))
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py

"""

import cvxpy as cp  
import numpy as np


class ModelPredictiveControl:
    lane_width: float = 4.0
    vehicle_length: float = 5.0

    def __init__(self, headway_time, Q, R, Rd, time_horizon: int, dt):
        self.headway_time = headway_time
        self.dt = dt

        self.x_number = 3
        self.u_number = 1

        self.time_horizon = time_horizon
        self.Q = Q
        self.R = R
        self.Rd = Rd
        self.acceleration_constraint = 9.81

        self.ego_vehicle: np.ndarray = np.array([])
        self.ego_vehicle_acceleration = 0
        self.ego_current_lane: int = 0
        self.front_vehicle: np.ndarray = np.array([])
        self.front_vehicle_acceleration = 0.0

    def update(self, obs, vehicle_acceleration):
        self.update_ego_vehicle(obs, vehicle_acceleration)
        self.update_current_front_vehicle(obs)
        acceleration = self._update_mpc()
        return acceleration

    def update_ego_vehicle(self, obs, vehicle_acceleration):
        self.ego_vehicle = obs[0]
        self.ego_vehicle_acceleration = vehicle_acceleration
        self.ego_current_lane = self.find_current_lane(self.ego_vehicle)

    def update_current_front_vehicle(self, obs):
        # 전방 차량
        front_vehicles = [vehicle for vehicle in obs[1:] if vehicle[0] == 1 and vehicle[1] >= self.ego_vehicle[1]]

        current_front_vehicles = [vehicle for vehicle in front_vehicles if
                                  self.find_current_lane(vehicle) == self.ego_current_lane]

        previous_front_vehicle_velocity = self.front_vehicle[3] if self.front_vehicle.size != 0 else None
        self.front_vehicle = self.find_closest_vehicle(current_front_vehicles)
        if previous_front_vehicle_velocity is not None:
            self.front_vehicle_acceleration = (self.front_vehicle[3] - previous_front_vehicle_velocity) / self.dt
        else:
            self.front_vehicle_acceleration = 0.0

    def find_current_lane(self, vehicle) -> int:
        return int((vehicle[2] + self.lane_width / 2) // self.lane_width) + 1

    def find_closest_vehicle(self, vehicles: list) -> np.ndarray:
        if not vehicles:
            return np.array([])

        return min(vehicles, key=lambda vehicle: abs(vehicle[1] - self.ego_vehicle[1]))

    def _update_mpc(self):
        x0 = self._get_state()

        x = cp.Variable((self.x_number, self.time_horizon + 1))
        u = cp.Variable((self.u_number, self.time_horizon))

        xref = np.zeros((self.x_number, self.time_horizon + 1))

        cost = 0.0
        constraints = []
        for t in range(self.time_horizon - 1):
            cost += cp.quad_form(xref[:, t + 1] - x[:, t + 1], self.Q)
            cost += cp.quad_form(u[:, t], self.R)
            cost += cp.quad_form(u[:, t + 1] - u[:, t], self.Rd)

            A, B, C = self._get_dynamics_model()
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + (C @ [self.front_vehicle_acceleration]).A1]

        constraints += [x[:, 0] == x0]
        constraints += [cp.abs(u[0, :]) <= self.acceleration_constraint]

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.ECOS, verbose=False)

        if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
            acceleration_inputs = np.array(u.value[0, :]).flatten()

        else:
            print("Fail to find optimal solution.")
            acceleration_inputs = [0]

        acceleration = acceleration_inputs[0]
        return acceleration

    def _get_state(self):
        relative_distance = self.front_vehicle[1] - self.ego_vehicle[1] - self.vehicle_length
        target_distance = self.ego_vehicle[3] * self.headway_time
        distance_error = target_distance - relative_distance
        velocity_error = self.front_vehicle[3] - self.ego_vehicle[3]

        return [distance_error, velocity_error, self.ego_vehicle_acceleration]

    def _get_dynamics_model(self):
        time_constant = 0.45
        A_c = np.matrix([
            [0, -1, self.headway_time],
            [0, 0, -1],
            [0, 0, -1 / time_constant]
        ])

        B_c = np.matrix([
            [0],
            [0],
            [1 / time_constant]
        ])

        C_c = np.matrix([
            [0],
            [1],
            [0]
        ])

        A = np.eye(self.x_number) + self.dt * A_c
        B = self.dt * B_c
        C = self.dt * C_c

        return A, B, C
