"""
Behavior Decision Making for Lane Change
Reference: J. Nilsson, J. Silvlin, M. Brannstrom, E. Coelingh, and J. Fredriksson,
           “If, When, and How to Perform Lane Change Maneuvers on Highways,”
           IEEE Intell. Transp. Syst. Mag., vol. 8, no. 4, pp. 68–78, 2016.
"""

from enum import Enum

import numpy as np


class BehaviorState(Enum):
    LEFT_LANE_CHANGE = 0
    LANE_KEEPING = 1
    RIGHT_LANE_CHANGE = 2
    CAR_FOLLOWING = 3


class BehaviorDecisionMaking:
    vehicle_length: float = 5.0
    lane_width: float = 4.0

    def __init__(self, desired_velocity, desired_headway_time, lane_range):
        self.desired_velocity = desired_velocity

        self.previous_behavior_state: BehaviorState = BehaviorState.LANE_KEEPING
        self.behavior_state: BehaviorState = BehaviorState.LANE_KEEPING

        self.lane_range = lane_range
        self.target_lane = 0

        self.beta = 300
        self.zero_division = 2

        self.alpha = 2
        self.desired_headway_time = desired_headway_time

        self.w1 = [5, 12]
        self.w2 = 0.5

        self.lane_change_threshold = 0.9
        self.right_lane_change = 0.05

        self.ego_vehicle: np.ndarray = np.array([])
        self.ego_current_lane = 0

        self.current_front_vehicle: np.ndarray = np.array([])
        self.current_rear_vehicle: np.ndarray = np.array([])
        self.left_front_vehicle: np.ndarray = np.array([])
        self.left_rear_vehicle: np.ndarray = np.array([])
        self.right_front_vehicle: np.ndarray = np.array([])
        self.right_rear_vehicle: np.ndarray = np.array([])

        self.u_ego = 0
        self.u_left = 0
        self.u_right = 0

        self.lane_change_time = 0

    def update(self, vehicles, dt):
        self.set_state(vehicles)

        self.previous_behavior_state = self.behavior_state
        if self.behavior_state in [BehaviorState.LANE_KEEPING, BehaviorState.CAR_FOLLOWING]:
            self.behavior_state = self.evaluate_lane_change()
            if (self.behavior_state == BehaviorState.LANE_KEEPING
                    and self.decide_car_following(self.current_front_vehicle)):
                self.behavior_state = BehaviorState.CAR_FOLLOWING
        elif self.behavior_state in [BehaviorState.LEFT_LANE_CHANGE, BehaviorState.RIGHT_LANE_CHANGE]:
            self.lane_change_time += dt
            if self.ego_current_lane == self.target_lane and abs(
                    self.ego_vehicle[2] - (self.target_lane - 1) * self.lane_width) <= 0.1:
                self.lane_change_time = 0.0
                if self.decide_car_following(self.current_front_vehicle):
                    self.behavior_state = BehaviorState.CAR_FOLLOWING
                else:
                    self.behavior_state = BehaviorState.LANE_KEEPING

    def set_state(self, vehicles):
        self.ego_vehicle: np.ndarray = vehicles[0]
        self.current_front_vehicle: np.ndarray = vehicles[1]
        self.current_rear_vehicle: np.ndarray = vehicles[2]
        self.left_front_vehicle: np.ndarray = vehicles[3]
        self.left_rear_vehicle: np.ndarray = vehicles[4]
        self.right_front_vehicle: np.ndarray = vehicles[5]
        self.right_rear_vehicle: np.ndarray = vehicles[6]

        self.ego_current_lane = self.find_current_lane(self.ego_vehicle)

    def find_current_lane(self, vehicle) -> int:
        return int((vehicle[2] + self.lane_width / 2) // self.lane_width) + 1

    def evaluate_lane_change(self):
        # TODO: MOBIL 등의 다양한 차선 변경 모델을 기반하여 차선 변경 결정을 할 수 있음
        self.u_ego = self.calculate_utility_function(self.current_front_vehicle, self.ego_vehicle)

        utility_right = self.calculate_utility_function(self.right_front_vehicle, self.right_rear_vehicle)
        self.u_right = (utility_right + self.right_lane_change - (1 + self.lane_change_threshold) * abs(self.u_ego))

        utility_left = self.calculate_utility_function(self.left_front_vehicle, self.left_rear_vehicle)
        self.u_left = (utility_left - self.right_lane_change - (1 + self.lane_change_threshold) * abs(self.u_ego))

        if self.ego_current_lane == self.lane_range[0]:
            self.u_left = -np.inf
        elif self.ego_current_lane == self.lane_range[1]:
            self.u_right = -np.inf

        target_lane_diff = np.argmax([self.u_left, self.u_ego, self.u_right])
        self.target_lane = self.ego_current_lane + target_lane_diff - 1
        behavior_state = BehaviorState(target_lane_diff)

        # TODO: Utility Function 을 통하여 차선 변경 결정이 되더라도, 적절한 시점에 차선 변경을 수행할 수 있도록 로직 고도화 필요
        if behavior_state == BehaviorState.LEFT_LANE_CHANGE:
            if not self.check_safety([self.left_front_vehicle, self.left_rear_vehicle]):
                behavior_state = BehaviorState.LANE_KEEPING
        elif behavior_state == BehaviorState.RIGHT_LANE_CHANGE:
            if not self.check_safety([self.right_front_vehicle, self.right_rear_vehicle]):
                behavior_state = BehaviorState.LANE_KEEPING
        return behavior_state

    def calculate_utility_function(self, preceding, following):
        # Surrounding Vehicle 가 없는 경우 임의의 값으로 대체
        if preceding.size and following.size:
            average_velocity = (preceding[3] + following[3]) / 2
            average_time_gap = (preceding[1] - following[1] - self.vehicle_length) / following[3]
        elif preceding.size and not following.size:
            average_velocity = preceding[3]
            average_time_gap = self.alpha * self.desired_headway_time
        elif not preceding.size and following.size:
            average_velocity = following[3]
            average_time_gap = self.alpha * self.desired_headway_time
        else:
            average_velocity = self.ego_vehicle[3]
            average_time_gap = self.alpha * self.desired_headway_time

        d_max = self.desired_velocity * self.beta
        u_v = -abs(d_max / self.desired_velocity - d_max / max(self.zero_division, average_velocity))
        n_v = abs(d_max / self.desired_velocity - d_max / self.zero_division)
        if average_velocity > self.desired_velocity:
            w1 = self.w1[1]
        else:
            w1 = self.w1[0]

        u_tg = min(self.alpha * self.desired_headway_time, average_time_gap)
        n_tg = self.alpha * self.desired_headway_time
        return w1 * u_v / n_v + self.w2 * u_tg / n_tg

    def decide_car_following(self, front_vehicle: np.ndarray) -> bool:
        if not front_vehicle.size:
            return False

        return self.calculate_headway_time(front_vehicle, self.ego_vehicle) < self.desired_headway_time

    def calculate_headway_time(self, front_vehicle, rear_vehicle) -> float:
        relative_distance = front_vehicle[1] - rear_vehicle[1] - self.vehicle_length
        return relative_distance / rear_vehicle[3]

    def check_safety(self, vehicles):
        for vehicle in vehicles:
            if vehicle.size and abs(self.ego_vehicle[1] - vehicle[1]) < 2.5 * self.vehicle_length:
                return False
        return True

    def is_initialized_lane_change(self):
        return (self.previous_behavior_state in [BehaviorState.LANE_KEEPING, BehaviorState.CAR_FOLLOWING]
                and self.behavior_state in [BehaviorState.LEFT_LANE_CHANGE, BehaviorState.RIGHT_LANE_CHANGE])

    def initialize(self):
        self.behavior_state = BehaviorState.LANE_KEEPING
        self.previous_behavior_state = BehaviorState.LANE_KEEPING
        self.lane_change_time = 0.0


def main():
    behavior_decision_making = BehaviorDecisionMaking(
        desired_velocity=20,
        desired_headway_time=2.0,
        lane_range=[1, 4],
    )

    current_lane = 3
    lane_width = 4
    dt = 0.1
    vehicles = np.array([
        np.array([1, 0, (current_lane - 1) * lane_width, 15, 0, 0]),  # Ego Vehicle
        np.array([1, 50, (current_lane - 1) * lane_width, 10, 0, 0]),  # Ego Front Vehicle
        np.array([1, -50, (current_lane - 1) * lane_width, 15, 0, 0]),  # Ego Rear Vehicle
        np.array([1, 50, (current_lane - 2) * lane_width, 15, 0, 0]),  # Left Front Vehicle
        np.array([1, -50, (current_lane - 2) * lane_width, 15, 0, 0]),  # Left Rear Vehicle
        np.array([1, 50, current_lane * lane_width, 10, 0, 0]),  # Right Front Vehicle
        np.array([1, -50, current_lane * lane_width, 10, 0, 0]),  # Right Rear Vehicle
    ])
    behavior_decision_making.update(vehicles, dt)
    print(f"behavior state: {behavior_decision_making.behavior_state} / "
          f"current lane: {behavior_decision_making.ego_current_lane} / "
          f"target lane: {behavior_decision_making.target_lane}")
    print(f"u_ego: {behavior_decision_making.u_ego} / "
          f"u_left: {behavior_decision_making.u_left} / "
          f"u_right: {behavior_decision_making.u_right}")


if __name__ == "__main__":
    main()
