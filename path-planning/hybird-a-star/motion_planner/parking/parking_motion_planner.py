"""
Motion Planner Parking
"""

from enum import Enum

import numpy as np


class ParkingState(Enum):
    FREE_DRIVING = 0
    FIND_FINAL_PATH = 1
    FOLLOW_FINAL_PATH = 2
    STRAIGHT = 3
    END = 4


class ParkingMotionPlanner:
    def __init__(self, goal_point: np.array, is_front_direction):
        self.parking_state = ParkingState.FREE_DRIVING

        self.wheelbase = 5
        self.vehicle_width = 2
        self.rear_length = 2.5
        self.maximum_steering_angle = np.radians(40)

        self.rear_wheel_position: np.array = np.array([0.0, 0.0])
        self.rear_wheel_local_position: np.array = np.array([0.0, 0.0])
        self.heading: float = 0.0
        self.local_heading: float = 0.0
        self.velocity: float = 0.0

        self.goal_point: np.array = np.array([goal_point[0], -goal_point[1]])
        self.parking_width = 5
        self.parking_length = 8
        self.parking_angle = np.radians(0.0)

        self.is_front_direction = is_front_direction

        self.r_turn = self.wheelbase / np.tan(self.maximum_steering_angle)
        self.r_min = self.r_turn - self.vehicle_width / 2
        if self.is_front_direction:
            self.r_max = np.sqrt((self.r_turn + self.vehicle_width / 2) ** 2 + self.wheelbase ** 2)
        else:
            self.r_max = self.r_turn + self.vehicle_width / 2

        self.x0 = self.goal_point[0]
        y0 = self.r_max ** 2 - (self.r_min + (self.parking_width / 2) ** 2) ** 2
        if y0 >= 0:
            self.y0 = np.sqrt(y0)
        else:
            self.y0 = -np.sqrt((self.vehicle_width * self.parking_width / 2) - (self.vehicle_width / 4) ** 2)
        self.origin_point = np.array([self.x0, self.y0 - goal_point[1] + self.parking_length / 2])

        self.final_path_radius = 0.0

    def update(self, obs):
        self._set_state(obs)

        # State Machine 조건 확인
        if self.parking_state == ParkingState.FREE_DRIVING:
            self.parking_state = self._check_free_driving()
        elif self.parking_state == ParkingState.FIND_FINAL_PATH:
            self.parking_state = self._check_find_final_path()
        elif self.parking_state == ParkingState.FOLLOW_FINAL_PATH:
            self.parking_state = self._check_follow_final_path()
        elif self.parking_state == ParkingState.STRAIGHT:
            self.parking_state = self._check_straight()
        elif self.parking_state == ParkingState.END:
            self.parking_state = self._check_end()

        # State Machine 실행
        if self.parking_state == ParkingState.FREE_DRIVING:
            control_command = self._run_free_driving()
        elif self.parking_state == ParkingState.FIND_FINAL_PATH:
            control_command = self._run_find_final_path()
        elif self.parking_state == ParkingState.FOLLOW_FINAL_PATH:
            control_command = self._run_follow_final_path()
        elif self.parking_state == ParkingState.STRAIGHT:
            control_command = self._run_straight()
        elif self.parking_state == ParkingState.END:
            control_command = self._run_end()

        return control_command

    def _set_state(self, obs):
        self.heading = -obs[4]
        self.local_heading = self._change_radians_range(self.heading - self.parking_angle)

        self.rear_wheel_position = np.array([
            obs[0] - self.rear_length * np.cos(self.heading),
            -obs[1] - self.rear_length * np.sin(self.heading)
        ])
        position_diff = self.rear_wheel_position - self.origin_point
        self.rear_wheel_local_position = np.array([
            np.cos(-self.parking_angle) * position_diff[0] - np.sin(-self.parking_angle) * position_diff[1],
            np.sin(-self.parking_angle) * position_diff[0] + np.cos(-self.parking_angle) * position_diff[1]
        ])

        velocity_direction = np.arctan2(-obs[3], obs[2])
        if abs(velocity_direction - self.heading) < np.pi / 2:
            self.velocity = np.sqrt(obs[2] ** 2 + obs[3] ** 2)
        else:
            self.velocity = -np.sqrt(obs[2] ** 2 + obs[3] ** 2)

    def _check_free_driving(self) -> ParkingState:
        is_parallel = np.pi / 4 <= abs(self.local_heading) <= np.pi - np.pi / 4

        if is_parallel:
            if abs(self.rear_wheel_local_position[1] + self.y0 - self.r_turn - self.wheelbase) >= 0.1:
                return ParkingState.FREE_DRIVING
            else:
                print("STATE: FIND_FINAL_PATH")
                return ParkingState.FIND_FINAL_PATH
        else:
            if self.is_front_direction:
                trigger_x_position = - 0.5 * self.r_turn - self.wheelbase
            else:
                trigger_x_position = 0.5 * self.r_turn
            if abs(self.rear_wheel_local_position[0] - trigger_x_position) >= 0.1:
                return ParkingState.FREE_DRIVING
            else:
                print("STATE: FIND_FINAL_PATH")
                return ParkingState.FIND_FINAL_PATH

    def _check_find_final_path(self) -> ParkingState:
        if self.final_path_radius:
            if not self.is_front_direction and self._is_stopped():
                print("STATE: FOLLOW_FINAL_PATH")
                return ParkingState.FOLLOW_FINAL_PATH
            else:
                print("STATE: FOLLOW_FINAL_PATH")
                return ParkingState.FOLLOW_FINAL_PATH

        else:
            return ParkingState.FIND_FINAL_PATH

    def _check_follow_final_path(self) -> ParkingState:
        if self.is_front_direction:
            heading_error = self._change_radians_range(self.local_heading - 1.5 * np.pi)
        else:
            heading_error = self._change_radians_range(self.local_heading - 0.5 * np.pi)

        if abs(heading_error) < 0.01:
            print("STATE: STRAIGHT")
            return ParkingState.STRAIGHT
        else:
            return ParkingState.FOLLOW_FINAL_PATH

    def _check_straight(self) -> ParkingState:
        if self.is_front_direction:
            if self.rear_wheel_local_position[1] - self.y0 - self.rear_length <= 0:
                print("STATE: END")
                return ParkingState.END
            else:
                return ParkingState.STRAIGHT
        else:
            if self.rear_wheel_local_position[1] - self.y0 + self.rear_length <= 0:
                print("STATE: END")
                return ParkingState.END
            else:
                return ParkingState.STRAIGHT

    def _check_end(self) -> ParkingState:
        return ParkingState.END

    def _run_free_driving(self):
        acceleration = self._control_velocity(1.5)
        return [acceleration, 0.0]

    def _run_find_final_path(self):
        x = self.rear_wheel_local_position[0]
        y = self.rear_wheel_local_position[1]
        r = abs((x ** 2 + y ** 2) / (2 * x))

        if x > 0:
            if abs(x) > r:
                theta_final = np.arcsin(y / r) + np.pi / 2
            else:
                theta_final = -np.arcsin(y / r) - np.pi / 2
        else:
            if abs(x) > r:
                theta_final = - np.arcsin(y / r) + np.pi / 2
            else:
                theta_final = np.arcsin(y / r) - np.pi / 2

        if self.is_front_direction:
            theta_error = self._change_radians_range(theta_final - self.local_heading)
        else:
            theta_error = self._change_radians_range(theta_final - self.local_heading - np.pi)

        if r > self.r_turn and (abs(theta_error) <= 0.005):
            if self.is_front_direction:
                self.final_path_radius = -np.sign(x) * r
                return [0, 0]
            else:
                self.final_path_radius = np.sign(x) * r
                return [self._control_to_stop(), 0]
        else:
            if theta_error > 0:
                return [0, -self.maximum_steering_angle]
            elif theta_error < 0:
                return [0, self.maximum_steering_angle]

    def _run_follow_final_path(self):
        if self.is_front_direction:
            acceleration = self._control_velocity(1.5)
        else:
            acceleration = self._control_velocity(-1.5)
        steering_angle = np.arctan(self.wheelbase / self.final_path_radius)
        return [acceleration, steering_angle]

    def _run_straight(self):
        if self.is_front_direction:
            acceleration = self._control_velocity(1.5)
        else:
            acceleration = self._control_velocity(-1.5)
        return [acceleration, 0]

    def _run_end(self):
        return [self._control_to_stop(), 0]

    def _control_velocity(self, target_velocity):
        if abs(self.velocity) < abs(target_velocity):
            acceleration = np.sign(target_velocity) * 0.5
        else:
            acceleration = 0.0
        return acceleration

    def _control_to_stop(self):
        if self.velocity > 0.1:
            acceleration = -9.81
        elif self.velocity < -0.1:
            acceleration = 9.81
        else:
            acceleration = 0.0
        return acceleration

    def _is_stopped(self):
        return abs(self.velocity) < 0.1

    @staticmethod
    # Imitation Code: https://stackoverflow.com/a/29237626
    # Return radians range from -pi to pi
    def _change_radians_range(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
