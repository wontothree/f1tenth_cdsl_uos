"""
Behavior Decision-Making and Lateral Motion Planner Module (Practice)
"""

import sys
from enum import Enum

import numpy as np

from src.motion_planner.lane_change.lane_change_trajectory_planner_practice import LaneChangeTrajectoryPlanner


class BehaviorState(Enum):
    LANE_KEEPING = 0
    CAR_FOLLOWING = 1
    LEFT_LANE_CHANGE = 2


class BehaviorDecisionMakingAndMotionPlanner:
    lane_width: float = 4.0
    vehicle_length: float = 5.0
    wheelbase: float = 5.0

    def __init__(self, sampling_time, target_headway_time=2, safety_ttc=4):
        self.sampling_time = sampling_time
        self.target_headway_time: float = target_headway_time
        self.safety_ttc: float = safety_ttc

        self.ego_vehicle: np.ndarray = np.array([])
        self.ego_current_lane: int = 0

        self.current_front_vehicle: np.ndarray = np.array([])
        self.left_front_vehicle: np.ndarray = np.array([])
        self.left_rear_vehicle: np.ndarray = np.array([])

        self.behavior_state: BehaviorState = BehaviorState.LANE_KEEPING

        self.target_speed = 20

        self.lane_change_motion_planner = None

    def update(self, obs, step):
        self.update_ego_vehicle(obs)
        self.update_surrounding_vehicles(obs)

        self.decide_behavior()
        control_command = self.plan_control_command()
        print(f"[{step * self.sampling_time:.2f} sec] position {self.ego_vehicle[1]:.2f} {-self.ego_vehicle[2]:.2f} / "
              f"velocity {self.ego_vehicle[3]:.2f} / steering {-np.degrees(control_command[1])} / {self.behavior_state}")

        return control_command

    def update_ego_vehicle(self, obs):
        self.ego_vehicle = obs[0]
        self.ego_current_lane = self.find_current_lane(self.ego_vehicle)

    def update_surrounding_vehicles(self, obs):
        # 전방 차량
        front_vehicles = [vehicle for vehicle in obs[1:] if vehicle[0] == 1 and vehicle[1] >= self.ego_vehicle[1]]

        current_front_vehicles = [vehicle for vehicle in front_vehicles if
                                  self.find_current_lane(vehicle) == self.ego_current_lane]
        self.current_front_vehicle = self.find_closest_vehicle(current_front_vehicles)

        left_front_vehicles = [vehicle for vehicle in front_vehicles if
                               self.find_current_lane(vehicle) == self.ego_current_lane - 1]
        self.left_front_vehicle = self.find_closest_vehicle(left_front_vehicles)

        # 후방 차량
        rear_vehicles = [vehicle for vehicle in obs[1:] if vehicle[0] == 1 and vehicle[1] < self.ego_vehicle[1]]

        left_rear_vehicles = [vehicle for vehicle in rear_vehicles if
                              self.find_current_lane(vehicle) == self.ego_current_lane - 1]
        self.left_rear_vehicle = self.find_closest_vehicle(left_rear_vehicles)

    def find_current_lane(self, vehicle) -> int:
        return int((vehicle[2] + self.lane_width / 2) // self.lane_width) + 1

    def find_closest_vehicle(self, vehicles: list) -> np.ndarray:
        if not vehicles:
            return np.array([])

        return min(vehicles, key=lambda vehicle: abs(vehicle[1] - self.ego_vehicle[1]))

    # TODO (1): Lane Change State 로 전환될 때, Lane Change Planner 생성
    def decide_behavior(self):
        """현재 차량의 주행 상황에 따라 Behavior 를 결정"""
        if self.behavior_state == BehaviorState.LANE_KEEPING:
            if self.decide_lane_change():
                self.behavior_state = BehaviorState.LEFT_LANE_CHANGE
            elif self.decide_reducing_speed(self.current_front_vehicle):
                self.behavior_state = BehaviorState.CAR_FOLLOWING

        elif self.behavior_state == BehaviorState.CAR_FOLLOWING:
            if self.decide_lane_change():
                self.behavior_state = BehaviorState.LEFT_LANE_CHANGE
            elif not self.decide_reducing_speed(self.current_front_vehicle):
                self.behavior_state = BehaviorState.LANE_KEEPING

        elif self.behavior_state == BehaviorState.LEFT_LANE_CHANGE:
            if self.ego_current_lane == 1 and self.lane_change_motion_planner.is_end():
                self.lane_change_motion_planner = None
                if self.decide_reducing_speed(self.current_front_vehicle):
                    self.behavior_state = BehaviorState.CAR_FOLLOWING
                else:
                    self.behavior_state = BehaviorState.LANE_KEEPING

    def decide_lane_change(self):
        """현재 차선의 위치와, 주행 효율, 안전을 고려하여 차선 변경 여부를 결정"""
        return self.ego_current_lane > 1 and self.decide_discretionary_lane_change() and self.check_lane_change_safety()

    def decide_reducing_speed(self, front_vehicle: np.ndarray) -> bool:
        """threshold headway time 보다 짧은 경우 감속하도록 판단"""
        if not front_vehicle.size:
            return False

        return self.calculate_headway_time(front_vehicle, self.ego_vehicle) < self.target_headway_time

    def decide_discretionary_lane_change(self) -> bool:
        """MOBIL 을 단순화 하여, 현재 차선의 선행 차량이 감속하고, 왼쪽 차선의 선행 차량이 감속하지 않는 경우 차선 변경"""
        is_reducing_speed_in_current_lane = self.decide_reducing_speed(self.current_front_vehicle)
        is_reducing_speed_in_left_lane = self.decide_reducing_speed(self.left_front_vehicle)
        return is_reducing_speed_in_current_lane and not is_reducing_speed_in_left_lane

    def check_lane_change_safety(self):
        """왼쪽 차선의 후방 차량과 전방 차량과의 ttc 가 safety ttc 보다 큰 경우 차선 변경"""
        if self.left_rear_vehicle.size:
            is_safe_rear = self.calculate_ttc(self.ego_vehicle, self.left_rear_vehicle) > self.safety_ttc
        else:
            is_safe_rear = True

        if self.left_front_vehicle.size:
            is_safe_front = self.calculate_ttc(self.left_front_vehicle, self.ego_vehicle) > self.safety_ttc
        else:
            is_safe_front = True
        return is_safe_rear and is_safe_front

    def calculate_headway_time(self, front_vehicle, rear_vehicle) -> float:
        relative_distance = front_vehicle[1] - rear_vehicle[1] - self.vehicle_length
        return relative_distance / rear_vehicle[3]

    def calculate_ttc(self, front_vehicle, rear_vehicle) -> float:
        relative_distance = front_vehicle[1] - rear_vehicle[1] - self.vehicle_length
        relative_velocity = rear_vehicle[3] - front_vehicle[3]
        if relative_distance <= 0:
            # 충돌 상황으로 ttc 를 0 으로 설정
            return 0
        elif relative_velocity <= 0:
            # 후방 차량이 전방 차량보다 속도가 느리므로, 충돌하지 않으므로 최대값으로 설정
            return sys.float_info.max
        else:
            return relative_distance / relative_velocity

    # TODO (2): Control Command 생성
    def plan_control_command(self):
        """Behavior 에 따라 Action Command 를 결정"""
        acceleration = 0.0
        steering_angle = 0.0

        return [acceleration, steering_angle]
