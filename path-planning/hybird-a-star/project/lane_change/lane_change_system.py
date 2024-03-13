"""
Lane Change System
"""

import numpy as np

from src.project.lane_change.behavior_decision_making import BehaviorDecisionMaking, BehaviorState
from src.project.lane_change.lane_change_motion_planner import LaneChangeMotionPlanner
from src.project.lane_change.lqr_car_following_controller import LqrCarFollowingController
from src.project.lane_change.pid_speed_controller import PidSpeedController
from src.project.lane_change.pure_pursuit_path_tracking_controller import PurePursuitPathTrackingController


class LaneChangeSystem:
    lane_width: float = 4.0
    vehicle_length: float = 5.0
    wheelbase: float = 5.0

    def __init__(self, env, sampling_time, desired_speed=20, desired_headway_time=2, lane_range=[1, 4]):
        self.sampling_time = sampling_time

        self.desired_speed = desired_speed
        self.desired_headway_time: float = desired_headway_time

        self.ego_vehicle: np.ndarray = np.array([])
        self.ego_current_lane: int = 0

        self.current_front_vehicle: np.ndarray = np.array([])
        self.current_rear_vehicle: np.ndarray = np.array([])
        self.left_front_vehicle: np.ndarray = np.array([])
        self.left_rear_vehicle: np.ndarray = np.array([])
        self.right_front_vehicle: np.ndarray = np.array([])
        self.right_rear_vehicle: np.ndarray = np.array([])

        self.behavior_decision_making = BehaviorDecisionMaking(desired_velocity=self.desired_speed,
                                                               desired_headway_time=self.desired_headway_time,
                                                               lane_range=lane_range)
        self.lane_change_motion_planner = LaneChangeMotionPlanner(time_horizon=20, dt=0.5,
                                                                  desired_speed=self.desired_speed)
        self.path_tracking_controller = PurePursuitPathTrackingController(env.unwrapped, gain=0.7, min_distance=5)

        self.speed_controller = PidSpeedController(gain=1)
        self.car_following_controller = LqrCarFollowingController(
            headway_time=self.desired_headway_time,
            Q=np.diag([2, 4]),
            R=np.diag([5]),
            dt=sampling_time
        )

    def update(self, obs, step):
        self.update_ego_vehicle(obs)
        self.update_surrounding_vehicles(obs)

        self.decide_behavior()
        control_command = self.plan_control_command()
        print(f"[{step * self.sampling_time:.2f} sec] position {self.ego_vehicle[1]:.2f} {-self.ego_vehicle[2]:.2f} / "
              f"velocity {self.ego_vehicle[3]:.2f} / acceleration {control_command[0]:.2f} / "
              f"steering {-np.degrees(control_command[1]):.2f} / {self.behavior_decision_making.behavior_state}")

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
                               self.find_current_lane(vehicle) == int(self.ego_current_lane - 1)]
        self.left_front_vehicle = self.find_closest_vehicle(left_front_vehicles)

        right_front_vehicles = [vehicle for vehicle in front_vehicles if
                                self.find_current_lane(vehicle) == int(self.ego_current_lane + 1)]
        self.right_front_vehicle = self.find_closest_vehicle(right_front_vehicles)

        # 후방 차량
        rear_vehicles = [vehicle for vehicle in obs[1:] if vehicle[0] == 1 and vehicle[1] < self.ego_vehicle[1]]
        current_rear_vehicles = [vehicle for vehicle in rear_vehicles if
                                 self.find_current_lane(vehicle) == self.ego_current_lane]
        self.current_rear_vehicle = self.find_closest_vehicle(current_rear_vehicles)

        left_rear_vehicles = [vehicle for vehicle in rear_vehicles if
                              self.find_current_lane(vehicle) == int(self.ego_current_lane - 1)]
        self.left_rear_vehicle = self.find_closest_vehicle(left_rear_vehicles)

        right_rear_vehicles = [vehicle for vehicle in rear_vehicles if
                               self.find_current_lane(vehicle) == int(self.ego_current_lane + 1)]
        self.right_rear_vehicle = self.find_closest_vehicle(right_rear_vehicles)

    def find_current_lane(self, vehicle) -> int:
        return int((vehicle[2] + self.lane_width / 2) // self.lane_width) + 1

    def find_closest_vehicle(self, vehicles: list) -> np.ndarray:
        if not vehicles:
            return np.array([])

        return min(vehicles, key=lambda vehicle: abs(vehicle[1] - self.ego_vehicle[1]))

    def decide_behavior(self):
        """현재 차량의 주행 상황에 따라 Behavior 를 결정"""
        self.behavior_decision_making.update(vehicles=[
            self.ego_vehicle,
            self.current_front_vehicle,
            self.current_rear_vehicle,
            self.left_front_vehicle,
            self.left_rear_vehicle,
            self.right_front_vehicle,
            self.right_rear_vehicle,
        ], dt=self.sampling_time)

        if self.behavior_decision_making.is_initialized_lane_change():
            if self.behavior_decision_making.behavior_state == BehaviorState.RIGHT_LANE_CHANGE:
                obs = [self.ego_vehicle, self.right_front_vehicle, self.right_rear_vehicle,
                       self.current_front_vehicle, self.current_rear_vehicle]
            else:
                obs = [self.ego_vehicle, self.left_front_vehicle, self.left_rear_vehicle,
                       self.current_front_vehicle, self.current_rear_vehicle]
            self.lane_change_motion_planner.update(vehicles=obs, target_lane=self.behavior_decision_making.target_lane)

            if not self.lane_change_motion_planner.has_trajectory():
                self.behavior_decision_making.initialize()

    def plan_control_command(self):
        if self.behavior_decision_making.behavior_state == BehaviorState.LANE_KEEPING:
            acceleration = self.speed_controller.update(self.desired_speed, self.ego_vehicle)
            steering_angle = self.path_tracking_controller.update(self.ego_vehicle)

        elif self.behavior_decision_making.behavior_state == BehaviorState.CAR_FOLLOWING:
            acceleration = self.car_following_controller.update(self.ego_vehicle, self.current_front_vehicle)
            steering_angle = self.path_tracking_controller.update(self.ego_vehicle)

        else:
            acceleration = self.lane_change_motion_planner.get_target_acceleration(
                self.behavior_decision_making.lane_change_time
            )
            steering_angle = self.path_tracking_controller.update(
                self.ego_vehicle,
                self.lane_change_motion_planner.longitudinal_trajectory,
                self.lane_change_motion_planner.lateral_trajectory
            )

        return [acceleration, steering_angle]
