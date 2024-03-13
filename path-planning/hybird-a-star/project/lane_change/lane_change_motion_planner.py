"""
Lane Change Motion Planning Using Convex Optimization
Reference: J. Nilsson, M. Brannstrom, J. Fredriksson, and E. Coelingh,
           “Longitudinal and Lateral Control for Automated Yielding Maneuvers,”
           IEEE Trans. Intell. Transp. Syst., vol. 17, no. 5, pp. 1404–1414, May 2016.
"""

import cvxpy as cp
import numpy as np

from src.project.lane_change.constant_velocity_prediction import ConstantVelocityPrediction


class LaneChangeMotionPlanner:
    lane_width = 4.0

    def __init__(self, dt, time_horizon, desired_speed):
        self.dt = dt
        self.time_horizon = time_horizon

        self.peri_horizon = 0
        self.post_horizon = 0

        self.v_des = desired_speed
        self.a_ymax = 3

        self.v = np.diag([0, 1])
        self.k = np.diag([1])
        self.phi = np.diag([0, 1])
        self.psi = np.diag([10])

        self.ego = None
        self.ego_current_lane = 0
        self.s1 = None
        self.s2 = None
        self.s3 = None
        self.s4 = None
        self.target_lane = 0

        self.x_limit = []
        self.y_limit = []
        self.vx_limit = [0, 30]
        self.vy_limit = [-2, 2]
        self.ax_limit = [-4, 2]
        self.ay_limit = [-2, 2]
        self.ax_rate_limit = [-3 * self.dt, 1.5 * self.dt]
        self.ay_rate_limit = [-0.5 * self.dt, 0.5 * self.dt]

        self.longitudinal_trajectory = None
        self.vxs = None
        self.axs = None
        self.lateral_trajectory = None
        self.vys = None
        self.ays = None

        self.headway_time = 0.5
        self.minimum_spacing = 0.5

        # TODO: Constant Velocity Model 보다 고도화된 Model 을 활용하여 Prediction 성능 향상
        self.prediction_model = ConstantVelocityPrediction(self.time_horizon, self.dt)

        self.A = np.matrix([[1, self.dt], [0, 1]])
        self.B = np.matrix([[0], [self.dt]])

    def update(self, vehicles, target_lane):
        self.set_vehicles(vehicles)
        self.target_lane = target_lane

        # TODO: Longitudinal 과 Lateral 을 결합하여 Trajectory 를 생성하는 방법 고려해보거나,
        #       현재의 Cost Function 과 Constraint 에 추가적인 요소를 고려하여 고도화
        self.determine_longitudinal_safety_corridor()
        self.determine_longitudinal_trajectory()
        if self.longitudinal_trajectory.size:
            self.determine_lateral_safety_corridor()
            self.determine_lateral_trajectory()

        return self.longitudinal_trajectory, self.lateral_trajectory

    def set_vehicles(self, vehicles):
        self.ego = vehicles[0]
        self.s1 = self.prediction_model.predict(vehicles[1])
        self.s2 = self.prediction_model.predict(vehicles[2])
        self.s3 = self.prediction_model.predict(vehicles[3])
        self.s4 = self.prediction_model.predict(vehicles[4])

        self.ego_current_lane = self.find_current_lane(self.ego)

    def find_current_lane(self, vehicle) -> int:
        return int((vehicle[2] + self.lane_width / 2) // self.lane_width) + 1

    def determine_longitudinal_safety_corridor(self):
        dy = abs((self.target_lane - self.ego_current_lane) * self.lane_width)
        t_min = - self.ego[4] / self.a_ymax + np.sqrt((self.ego[4] / self.a_ymax) ** 2 + 2 * dy / self.a_ymax)
        n_min = int(t_min / self.dt)
        self.post_horizon = n_min + self.peri_horizon

        self.x_limit = []

        for i in range(self.time_horizon + 1):
            s4_min = self.s4[i][1] + self.calculate_safety_distance(self.s4[i][3]) if self.s4.size else -np.inf
            s2_min = self.s2[i][1] + self.calculate_safety_distance(self.s2[i][3]) if self.s2.size else -np.inf

            s3_max = self.s3[i][1] - self.calculate_safety_distance(self.s3[i][3]) if self.s3.size else np.inf
            s1_max = self.s1[i][1] - self.calculate_safety_distance(self.s1[i][3]) if self.s1.size else np.inf

            if 0 <= i < self.peri_horizon:
                x_min = s4_min
                x_max = s3_max
            elif self.peri_horizon <= i < self.post_horizon:
                x_min = max(s4_min, s2_min)
                x_max = min(s1_max, s3_max)
            else:
                x_min = s2_min
                x_max = s1_max

            self.x_limit.append([x_min, x_max])

    def calculate_safety_distance(self, velocity):
        return max(self.minimum_spacing, velocity * self.headway_time)

    def determine_longitudinal_trajectory(self):
        x0 = [self.ego[1], self.ego[3]]

        x = cp.Variable((2, self.time_horizon + 1))
        u = cp.Variable((1, self.time_horizon))

        cost = 0.0
        constraints = []
        v_des = self.s1[0][3] if self.s1.size else self.v_des
        for t in range(self.time_horizon):
            cost += cp.quad_form(x[:, t + 1] - v_des, self.v)
            cost += cp.quad_form(u[:, t], self.k)

            constraints += [x[:, t + 1] == self.A @ x[:, t] + self.B @ u[:, t]]
            if self.x_limit[t + 1][1] != np.inf:
                constraints += [x[0, t + 1] <= self.x_limit[t + 1][1]]
            if self.x_limit[t + 1][0] != -np.inf:
                constraints += [x[0, t + 1] >= self.x_limit[t + 1][0]]

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
            print("Fail to find longitudinal optimal solution.")
            self.longitudinal_trajectory = np.array([])
            self.vxs = np.array([])
            self.axs = np.array([])

    def determine_lateral_safety_corridor(self):
        self.y_limit = []

        for i in range(self.time_horizon + 1):
            if 0 <= i < self.peri_horizon:
                y_min = (self.ego_current_lane - 1) * self.lane_width - self.lane_width / 2
                y_max = (self.ego_current_lane - 1) * self.lane_width + self.lane_width / 2
            elif self.peri_horizon <= i < self.post_horizon + 3:
                y_min = (min(self.target_lane, self.ego_current_lane) - 1) * self.lane_width - self.lane_width / 2
                y_max = (max(self.target_lane, self.ego_current_lane) - 1) * self.lane_width + self.lane_width / 2
            else:
                y_min = (self.target_lane - 1) * self.lane_width - self.lane_width / 2
                y_max = (self.target_lane - 1) * self.lane_width + self.lane_width / 2

            self.y_limit.append([y_min, y_max])

    def determine_lateral_trajectory(self):
        x0 = [self.ego[2], self.ego[4]]

        x = cp.Variable((2, self.time_horizon + 1))
        u = cp.Variable((1, self.time_horizon))

        cost = 0.0
        constraints = []
        for t in range(self.time_horizon):
            cost += cp.quad_form(x[:, t + 1], self.phi)
            cost += cp.quad_form(u[:, t], self.psi)

            constraints += [x[:, t + 1] == self.A @ x[:, t] + self.B @ u[:, t]]
            if self.y_limit[t + 1][1] != np.inf:
                constraints += [x[0, t + 1] <= self.y_limit[t + 1][1]]
            if self.y_limit[t + 1][0] != np.inf:
                constraints += [x[0, t + 1] >= self.y_limit[t + 1][0]]

            constraints += [x[1, t + 1] <= 0.17 * self.vxs[t + 1]]
            constraints += [x[1, t + 1] >= -0.17 * self.vxs[t + 1]]

            if t + 1 < self.time_horizon:
                constraints += [u[0, t + 1] - u[0, t] <= self.ay_rate_limit[1]]
                constraints += [u[0, t + 1] - u[0, t] >= self.ay_rate_limit[0]]

        constraints += [x[:, 0] == x0]
        constraints += [x[1, :] <= self.vy_limit[1]]
        constraints += [x[1, :] >= self.vy_limit[0]]
        constraints += [u[0, :] <= self.ay_limit[1]]
        constraints += [u[0, :] >= self.ay_limit[0]]

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.ECOS, verbose=False)

        if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
            self.lateral_trajectory = np.array(x.value[0, :]).flatten()
            self.vys = np.array(x.value[1, :]).flatten()
            self.ays = np.array(u.value[0, :]).flatten()

        else:
            print("Fail to find lateral optimal solution.")
            self.lateral_trajectory = np.array([])
            self.vys = np.array([])
            self.ays = np.array([])

    def has_trajectory(self):
        return not (self.longitudinal_trajectory.size == 0 or self.lateral_trajectory.size == 0)

    def get_target_acceleration(self, time):
        index = int(time / self.dt)
        return self.axs[index] if index < len(self.axs) else self.axs[-1]


def main():
    time_horizon = 20
    dt = 0.5
    lane_change_motion_planner = LaneChangeMotionPlanner(
        time_horizon=time_horizon,
        dt=dt,
        desired_speed=20
    )

    vehicles = np.array([
        np.array([1, 10, 0, 25, 0, 0]),
        np.array([1, 100, 4, 20, 0, 0]),
        np.array([1, -80, 4, 20, 0, 0]),
        np.array([1, 80, 0, 15, 0, 0]),
        np.array([1, -50, 0, 15, 0, 0]),
    ])
    target_lane = 2
    xs, ys = lane_change_motion_planner.update(vehicles, target_lane)

    import matplotlib.pyplot as plt
    times = np.arange(0, time_horizon * dt + dt, dt) if len(xs) != 1 else [0.0]
    fig = plt.figure(1)
    ax = fig.add_subplot(2, 1, 1)
    ax.plot(times[:-1], lane_change_motion_planner.axs)
    ax.grid(True)
    ax.set(xlim=[times[0], times[-1]], ylim=[-4, 4], ylabel='ax [m/s^2]')

    ax = fig.add_subplot(2, 1, 2)
    ax.plot(times[:-1], lane_change_motion_planner.ays)
    ax.grid(True)
    ax.set(xlim=[times[0], times[-1]], ylim=[-4, 4], xlabel='Time [s]', ylabel='ay [m/s^2]')

    fig = plt.figure(2)
    ax = fig.add_subplot(2, 1, 1)
    ax.plot(times, lane_change_motion_planner.vxs)
    ax.grid(True)
    ax.set(xlim=[times[0], times[-1]], ylim=[0, 30], ylabel='vx [m/s]')

    ax = fig.add_subplot(2, 1, 2)
    ax.plot(times, lane_change_motion_planner.vys)
    ax.grid(True)
    ax.set(xlim=[times[0], times[-1]], ylim=[-4, 4], xlabel='Time [s]', ylabel='vy [m/s]')

    fig = plt.figure(3)
    plt.plot(xs, ys, "b-o", label="trajectory")
    plt.plot(vehicles[0][1], vehicles[0][2], "yo", label="Ego Vehicle")
    plt.plot(vehicles[1:5, 1], vehicles[1:5, 2], "ro", label="Surrounding Vehicle")
    plt.plot([-100, 200], [-2, - 2], "k", label="Lane Constraint")
    plt.plot([-100, 200], [2, 2], "k--")
    plt.plot([-100, 200], [6, 6], "k")

    plt.grid(True)
    # plt.axis("equal")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.title("Planning Result")
    plt.show()


if __name__ == "__main__":
    main()
