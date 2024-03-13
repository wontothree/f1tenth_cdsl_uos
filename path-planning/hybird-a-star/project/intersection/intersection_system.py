"""
Intersection System
"""

import numpy as np

from src.project.intersection.behavior_decision_making import BehaviorDecisionMaking, BehaviorState
from src.project.intersection.intersection_localization import IntersectionLocalization
from src.project.intersection.intersection_motion_planner import IntersectionMotionPlanner
from src.project.intersection.lqr_path_tracking_controller import LqrPathTrackingController
from src.project.intersection.pid_speed_controller import PidSpeedController


class IntersectionSystem:
    lane_width: float = 4.0
    vehicle_length: float = 5.0
    wheelbase: float = 5.0

    def __init__(self, env, sampling_time, desired_speed):
        self.sampling_time = sampling_time

        self.desired_speed = desired_speed

        self.ego_vehicle: np.ndarray = np.array([])
        self.is_exit: bool = False
        self.is_in_intersection: bool = False
        self.lane_right_source_vehicles = []
        self.lane_right_sink_vehicles = []
        self.lane_left_source_vehicles = []
        self.lane_left_sink_vehicles = []
        self.sensor_range = 50

        self.behavior_decision_making = BehaviorDecisionMaking()
        self.intersection_motion_planner = IntersectionMotionPlanner(time_horizon=25, dt=0.2,
                                                                     desired_speed=self.desired_speed)

        self.path_tracking_controller = LqrPathTrackingController(env.unwrapped, Q=np.diag([10, 1, 10, 1]),
                                                                  R=np.diag([5]),
                                                                  dt=sampling_time)

        self.speed_controller = PidSpeedController(gain=1)

    def update(self, obs, step):
        self.update_ego_vehicle(obs)
        self.update_surrounding_vehicles(obs)

        self.decide_behavior()
        control_command = self.plan_control_command()
        print(f"[{step * self.sampling_time:.2f} sec] position {self.ego_vehicle[1]:.2f} {-self.ego_vehicle[2]:.2f} / "
              f"velocity {np.linalg.norm(self.ego_vehicle[3:5]):.2f} / acceleration {control_command[0]:.2f} / "
              f"steering {-np.degrees(control_command[1]):.2f} / {self.behavior_decision_making.behavior_state} /"
              f" {self.behavior_decision_making.risk_management_behavior_state if self.behavior_decision_making.behavior_state == BehaviorState.RISK_MANAGEMENT else ''}")

        return control_command

    def update_ego_vehicle(self, obs):
        self.ego_vehicle = obs[0]
        self.is_in_intersection = IntersectionLocalization.is_in_intersection(self.ego_vehicle)
        self.is_exit = IntersectionLocalization.is_in_up_sink(self.ego_vehicle)

    def update_surrounding_vehicles(self, obs):
        self.lane_right_source_vehicles = sorted([vehicle for vehicle in obs[1:] if
                                                  vehicle[0] == 1 and IntersectionLocalization.is_in_right_source(vehicle)],
                                                 key=lambda vehicle: abs(vehicle[1]))
        self.lane_right_sink_vehicles = [vehicle for vehicle in obs[1:] if
                                         vehicle[0] == 1 and IntersectionLocalization.is_in_right_sink(vehicle)]
        self.lane_left_source_vehicles = sorted([vehicle for vehicle in obs[1:] if
                                                 vehicle[0] == 1 and IntersectionLocalization.is_in_left_source(vehicle)],
                                                key=lambda vehicle: abs(vehicle[1]))
        self.lane_left_sink_vehicles = [vehicle for vehicle in obs[1:] if
                                        vehicle[0] == 1 and IntersectionLocalization.is_in_left_sink(vehicle)]

    def decide_behavior(self):
        """현재 차량의 주행 상황에 따라 Behavior 를 결정"""
        right_target_vehicles = [vehicle for vehicle in self.lane_right_source_vehicles if
                                 abs(vehicle[1]) <= self.sensor_range]
        left_target_vehicles = [vehicle for vehicle in self.lane_left_source_vehicles if
                                abs(vehicle[1]) <= self.sensor_range]
        self.behavior_decision_making.update(
            self.ego_vehicle,
            left_target_vehicles,
            right_target_vehicles,
            self.is_exit,
        )

    def plan_control_command(self):
        steering_angle = self.path_tracking_controller.update(self.ego_vehicle)
        if self.behavior_decision_making.behavior_state == BehaviorState.APPROACH:
            acceleration = self.speed_controller.update(self.desired_speed, self.ego_vehicle)

        elif self.behavior_decision_making.behavior_state == BehaviorState.RISK_MANAGEMENT:
            target_vehicles = [vehicle for vehicle in (self.lane_right_source_vehicles + self.lane_left_source_vehicles)
                               if abs(vehicle[1]) <= self.sensor_range]
            acceleration = self.intersection_motion_planner.update(
                self.ego_vehicle,
                target_vehicles,
                self.behavior_decision_making.risk_management_behavior_state
            )
        elif self.behavior_decision_making.behavior_state == BehaviorState.EXIT:
            acceleration = self.speed_controller.update(self.desired_speed, self.ego_vehicle)

        return [acceleration, steering_angle]
