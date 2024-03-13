"""
Car Following Control using PID Controller
"""

import numpy as np


class PidModule:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.integral = 0
        self.prev_error = 0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * derivative


class PidController:
    lane_width: float = 4.0
    vehicle_length: float = 5.0

    def __init__(self, headway_time, distance_gains, velocity_gains, dt):
        self.headway_time = headway_time
        self.dt = dt
        self.distance_pid = PidModule(distance_gains[0], distance_gains[1], distance_gains[2], self.dt)
        self.velocity_pid = PidModule(velocity_gains[0], velocity_gains[1], velocity_gains[2], self.dt)

        self.ego_vehicle: np.ndarray = np.array([])
        self.ego_current_lane: int = 0
        self.front_vehicle: np.ndarray = np.array([])

        self.distance_error = 0
        self.velocity_error = 0

    def update(self, obs, vehicle_acceleration):
        self.update_ego_vehicle(obs, vehicle_acceleration)
        self.update_current_front_vehicle(obs)

        relative_distance = self.front_vehicle[1] - self.ego_vehicle[1] - self.vehicle_length
        target_distance = self.ego_vehicle[3] * self.headway_time
        self.distance_error = target_distance - relative_distance
        self.velocity_error = self.front_vehicle[3] - self.ego_vehicle[3]
        acceleration = - self.distance_pid.update(self.distance_error) - self.velocity_pid.update(self.velocity_error)
        return acceleration

    def update_ego_vehicle(self, obs, vehicle_acceleration):
        self.ego_vehicle = obs[0]
        self.ego_current_lane = self.find_current_lane(self.ego_vehicle)

    def update_current_front_vehicle(self, obs):
        # 전방 차량
        front_vehicles = [vehicle for vehicle in obs[1:] if vehicle[0] == 1 and vehicle[1] >= self.ego_vehicle[1]]

        current_front_vehicles = [vehicle for vehicle in front_vehicles if
                                  self.find_current_lane(vehicle) == self.ego_current_lane]
        self.front_vehicle = self.find_closest_vehicle(current_front_vehicles)

    def find_current_lane(self, vehicle) -> int:
        return int((vehicle[2] + self.lane_width / 2) // self.lane_width) + 1

    def find_closest_vehicle(self, vehicles: list) -> np.ndarray:
        if not vehicles:
            return np.array([])

        return min(vehicles, key=lambda vehicle: abs(vehicle[1] - self.ego_vehicle[1]))
