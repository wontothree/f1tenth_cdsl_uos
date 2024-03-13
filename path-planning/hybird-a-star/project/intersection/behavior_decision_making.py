"""
Behavior Decision Making for Intersection
Reference: Y. Jeong and K. Yi,
“Target Vehicle Motion Prediction-Based Motion Planning Framework for Autonomous Driving in Uncontrolled Intersections,”
IEEE Trans. Intell. Transp. Syst., vol. 22, no. 1, pp. 168–177, Jan. 2021.
"""

from enum import Enum

import numpy as np

class BehaviorState(Enum):
    APPROACH = 0
    RISK_MANAGEMENT = 1
    EXIT = 2


class RiskManagementBehaviorState(Enum):
    CROSS = 0
    YIELD = 1


class BehaviorDecisionMaking:
    vehicle_length: float = 5.0
    lane_width: float = 4.0

    def __init__(self):
        self.behavior_state: BehaviorState = BehaviorState.APPROACH
        self.risk_management_behavior_state = RiskManagementBehaviorState.YIELD

        self.ego_vehicle: np.ndarray = np.array([])
        self.left_target_vehicles = []
        self.right_target_vehicles = []

        self.selected_target_vehicle = []

        self.right_conflict_point = np.array([2, 2])
        self.left_conflict_point = np.array([2, -2])

        self.t_critical = 4
        self.t_up = 1

    def update(self, ego_vehicle, left_target_vehicles, right_target_vehicles, is_exit):
        self.ego_vehicle = ego_vehicle
        # TODO: 직진 주행 뿐만 아니라, 좌회전, 우회전 등의 경우도 추가하여 더 많은 차선의 차량들 고려
        self.left_target_vehicles = left_target_vehicles
        self.right_target_vehicles = right_target_vehicles

        if self.behavior_state == BehaviorState.APPROACH:
            self.decide_intersection_behavior()

        elif self.behavior_state == BehaviorState.RISK_MANAGEMENT:
            if is_exit:
                self.behavior_state = BehaviorState.EXIT
            else:
                self.decide_intersection_behavior()

        elif self.behavior_state == BehaviorState.EXIT:
            pass

    def decide_intersection_behavior(self):
        is_empty = len(self.left_target_vehicles) == 0 and len(self.right_target_vehicles) == 0
        if is_empty:
            self.behavior_state = BehaviorState.APPROACH
        else:
            self.behavior_state = BehaviorState.RISK_MANAGEMENT

            is_cross = True
            if len(self.left_target_vehicles) == 1:
                is_cross &= self.calculate_primary_target_case(self.left_target_vehicles[0], is_left=True)
            elif len(self.left_target_vehicles) > 1:
                is_cross &= self.calculate_secondary_target_case(
                    self.left_target_vehicles[0], self.left_target_vehicles[1], is_left=True
                )

            if len(self.right_target_vehicles) == 1:
                is_cross &= self.calculate_primary_target_case(self.right_target_vehicles[0], is_left=False)
            elif len(self.right_target_vehicles) > 1:
                is_cross &= self.calculate_secondary_target_case(
                    self.right_target_vehicles[0], self.right_target_vehicles[1], is_left=False
                )

            if is_cross:
                self.risk_management_behavior_state = RiskManagementBehaviorState.CROSS
            else:
                self.risk_management_behavior_state = RiskManagementBehaviorState.YIELD

    def calculate_primary_target_case(self, target_vehicle, is_left):
        dtc_ego, dtc_sur = self.calculate_dtc(target_vehicle, is_left)

        ttc_ego = self.calculate_ttc(dtc_ego, abs(self.ego_vehicle[4]))

        v_sur = abs(target_vehicle[3])
        ttc_sur = self.calculate_ttc(dtc_sur, v_sur)

        if self.risk_management_behavior_state == RiskManagementBehaviorState.YIELD:
            rate = 1 / (ttc_ego + self.t_up)
        elif self.risk_management_behavior_state == RiskManagementBehaviorState.CROSS:
            rate = 1 / (ttc_ego - self.t_up)

        is_cross = 1 / ttc_sur <= rate
        return is_cross

    def calculate_secondary_target_case(self, primary_target_vehicle, secondary_target_vehicle, is_left):
        dtc_ego, dtc_primary_sur = self.calculate_dtc(primary_target_vehicle, is_left)
        dtc_ego, dtc_secondary_sur = self.calculate_dtc(secondary_target_vehicle, is_left)

        ttc_ego = self.calculate_ttc(dtc_ego, abs(self.ego_vehicle[4]))

        v_primary_sur = abs(primary_target_vehicle[3])
        ttc_primary_sur = self.calculate_ttc(dtc_primary_sur, v_primary_sur)

        v_secondary_sur = abs(secondary_target_vehicle[3])
        ttc_secondary_sur = self.calculate_ttc(dtc_secondary_sur, v_secondary_sur)

        if self.risk_management_behavior_state == RiskManagementBehaviorState.YIELD:
            rate = 1 / (ttc_primary_sur + self.t_critical + self.t_up)
        elif self.risk_management_behavior_state == RiskManagementBehaviorState.CROSS:
            rate = 1 / (ttc_primary_sur + self.t_critical - self.t_up)

        is_cross = 1 / ttc_secondary_sur <= rate
        return is_cross

    def calculate_dtc(self, target_vehicle, is_left):
        if is_left:
            dtc_ego = self.ego_vehicle[2] - self.left_conflict_point[1]
            dtc_sur = target_vehicle[1] - self.left_conflict_point[0]
        else:
            dtc_ego = self.ego_vehicle[2] - self.right_conflict_point[1]
            dtc_sur = self.right_conflict_point[0] - target_vehicle[1]

        return dtc_ego, dtc_sur

    @staticmethod
    def calculate_ttc(dtc, v):
        ttc = dtc / v
        return ttc


def main():
    ego_vehicle = np.array([1, 2, 20, 0, -10, np.radians(-90)])
    # Single target vehicle
    left_target_vehicles = [
        np.array([1, 100, -2, -20, 0, np.radians(180)]),
    ]
    right_target_vehicles = []

    # Multiple target vehicles
    left_target_vehicles = [
        np.array([1, 60, -2, -10, 0, np.radians(180)]),
        np.array([1, 80, -2, -10, 0, np.radians(180)]),
    ]
    right_target_vehicles = [
        np.array([1, -60, 2, 10, 0, np.radians(0)]),
        np.array([1, -80, 2, 10, 0, np.radians(0)]),
    ]

    behavior_decision_making = BehaviorDecisionMaking()
    behavior_decision_making.update(ego_vehicle, left_target_vehicles, right_target_vehicles, "up0")

    print(
        f"behavior: {behavior_decision_making.behavior_state} / "
        f"risk management: {behavior_decision_making.risk_management_behavior_state}"
    )


if __name__ == "__main__":
    main()
