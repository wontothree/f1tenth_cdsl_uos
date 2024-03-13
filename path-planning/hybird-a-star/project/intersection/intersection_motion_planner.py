"""
Intersection Motion Planning and Control Using MPC
Reference: Y. Jeong and K. Yi,
“Target Vehicle Motion Prediction-Based Motion Planning Framework for Autonomous Driving in Uncontrolled Intersections,”
IEEE Trans. Intell. Transp. Syst., vol. 22, no. 1, pp. 168–177, Jan. 2021.
"""

import cvxpy as cp
import numpy as np

from src.project.intersection.behavior_decision_making import RiskManagementBehaviorState
from src.project.intersection.constant_acceleration_prediction import ConstantAccelerationPrediction
from src.project.intersection.constant_velocity_prediction import ConstantVelocityPrediction


class IntersectionMotionPlanner:
    vehicle_length = 5.0

    def __init__(self, dt, time_horizon, desired_speed):
        self.dt = dt
        self.time_horizon = time_horizon

        self.v_des = desired_speed

        self.q = np.diag([0.25, 0.1, 0])
        self.r = np.diag([20])

        self.vx_limit = [0, 15]
        self.ax_limit = [-5, 3]
        self.ax_rate_limit = [-5 * self.dt, 5 * self.dt]

        self.longitudinal_trajectory = np.array([])
        self.vxs = np.array([])
        self.axs = np.array([])
        self.acceleration = 0.0

        self.c_min = 5
        self.ttc_min = 3

        # TODO: 더 정확한 예측 모델을 활용
        self.cv_model = ConstantVelocityPrediction(self.time_horizon, self.dt)
        self.ca_model = ConstantAccelerationPrediction(self.time_horizon, self.dt)

        self.tau = 0.5
        self.A = np.matrix([[1, self.dt, 0], [0, 1, self.dt], [0, 0, 1 - self.dt / self.tau]])
        self.B = np.matrix([[0], [0], [self.dt / self.tau]])

        self.ego_vehicle = np.array([])
        self.predicted_ego = np.array([])
        self.target_vehicle = np.array([])
        self.predicted_target = np.array([])
        self.risk_management_behavior_state = RiskManagementBehaviorState.YIELD
        self.target_x = 2.0

    def update(self, ego_vehicle, target_vehicles, risk_management_behavior_state):
        self.ego_vehicle = ego_vehicle
        self.risk_management_behavior_state = risk_management_behavior_state

        self.find_target_vehicle(target_vehicles)
        self.update_mpc()
        self.acceleration = self.axs[0]
        return self.acceleration

    def find_target_vehicle(self, target_vehicles):
        self.target_vehicle = np.array([])
        self.predicted_target = np.array([])

        self.predicted_ego = self.ca_model.predict(self.ego_vehicle, -self.acceleration)

        collision_time = self.time_horizon + 1
        for target_vehicle in target_vehicles:
            predicted_target = self.cv_model.predict(target_vehicle)
            for t in range(self.time_horizon + 1):
                if np.linalg.norm(
                        self.predicted_ego[t, 1:3] - predicted_target[t, 1:3]) <= self.vehicle_length * np.sqrt(2):
                    if t <= collision_time:
                        collision_time = t
                        self.target_vehicle = target_vehicle
                        self.predicted_target = predicted_target

    def update_mpc(self):
        x0 = [0.0, -self.ego_vehicle[4], self.acceleration]

        x = cp.Variable((3, self.time_horizon + 1))
        u = cp.Variable((1, self.time_horizon))

        xref = np.zeros((3, self.time_horizon + 1))
        for t in range(self.time_horizon + 1):
            xref[0, t] = -(self.predicted_ego[t, 2] - self.ego_vehicle[2])
            xref[1, t] = self.v_des
            xref[2, t] = 0
        cost = 0.0
        constraints = []
        for t in range(self.time_horizon):
            cost += cp.quad_form(x[:, t + 1] - xref[:, t + 1], self.q)
            cost += cp.quad_form(u[:, t], self.r)

            constraints += [x[:, t + 1] == self.A @ x[:, t] + self.B @ u[:, t]]

            if self.target_vehicle.size:
                c = (self.ttc_min - abs(self.predicted_target[t + 1, 1] - self.target_x) / abs(
                    self.predicted_target[t + 1, 3])) * x[1, t + 1] + self.c_min
            else:
                c = 0
            if self.risk_management_behavior_state == RiskManagementBehaviorState.CROSS:
                constraints += [x[0, t + 1] >= xref[0, t + 1] + c - self.vehicle_length]
            else:
                constraints += [x[0, t + 1] <= xref[0, t + 1] - c + self.vehicle_length]

            if t + 1 < self.time_horizon:
                constraints += [u[0, t + 1] - u[0, t] <= self.ax_rate_limit[1]]
                constraints += [u[0, t + 1] - u[0, t] >= self.ax_rate_limit[0]]

        constraints += [x[:, 0] == x0]
        constraints += [x[1, :] <= self.vx_limit[1]]
        constraints += [x[1, :] >= self.vx_limit[0]]
        constraints += [u[0, :] <= self.ax_limit[1]]
        constraints += [u[0, :] >= self.ax_limit[0]]

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.ECOS, verbose=False)

        if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
            self.longitudinal_trajectory = np.array(x.value[0, :]).flatten()
            self.vxs = np.array(x.value[1, :]).flatten()
            self.axs = np.array(u.value[0, :]).flatten()

        else:
            self.longitudinal_trajectory = np.array([x0])
            self.vxs = np.array([x0[1]])
            self.axs = np.array([self.acceleration])


def main():
    ego_vehicle = np.array([1, 2, 50, 0, -10, np.radians(-90)])
    # Single target vehicle
    left_target_vehicles = [
        np.array([1, 50, -2, -10, 0, np.radians(180)]),
    ]

    dt = 0.2
    time_horizon = 25
    intersection_motion_planner = IntersectionMotionPlanner(
        dt=dt,
        time_horizon=time_horizon,
        desired_speed=10
    )
    intersection_motion_planner.update(ego_vehicle, left_target_vehicles, RiskManagementBehaviorState.YIELD)

    import matplotlib.pyplot as plt
    times = np.arange(0, time_horizon * dt + dt, dt)
    fig = plt.figure(1)
    ax = fig.add_subplot(2, 1, 1)
    ax.plot(times[:-1], intersection_motion_planner.vxs[:-1])
    ax.grid(True)
    ax.set(xlim=[times[0], times[-2]], ylim=[0, 15], ylabel='vx [m/s]')

    ax = fig.add_subplot(2, 1, 2)
    ax.plot(times[:-1], intersection_motion_planner.axs)
    ax.grid(True)
    ax.set(xlim=[times[0], times[-2]], ylim=[-5, 5], xlabel='Time [s]', ylabel='ax [m/s^2]')

    fig = plt.figure(3)
    plt.plot(np.repeat(2, time_horizon + 1), intersection_motion_planner.longitudinal_trajectory - ego_vehicle[2],
             "b-o", label="trajectory")
    plt.plot(ego_vehicle[1], -ego_vehicle[2], "yo", label="Ego Vehicle")
    if intersection_motion_planner.predicted_target.size:
        plt.plot(intersection_motion_planner.predicted_target[:, 1],
                 -intersection_motion_planner.predicted_target[:, 2], "k-o", label="Predicted Target Vehicle")

    plt.plot(left_target_vehicles[0][1], -left_target_vehicles[0][2], "ro", label="Target Vehicle")
    plt.plot([-50, 50], [-4, -4], "k")
    plt.plot([-50, 50], [0, 0], "--k")
    plt.plot([-50, 50], [4, 4], "k")

    plt.plot([4, 4], [-50, 50], "k")
    plt.plot([0, 0], [-50, 50], "--k")
    plt.plot([-4, -4], [-50, 50], "k")

    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.title("Planning Result")
    plt.show()


if __name__ == "__main__":
    main()
