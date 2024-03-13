"""
Lane Change Trajectory Planner
"""

import numpy as np


class LaneChangeTrajectoryPlanner:
    def __init__(self, lane_change_time, initial_state, target_y, wheelbase):
        self.lane_change_time = lane_change_time
        self.initial_state = initial_state
        self.target_y = target_y
        self.wheelbase = wheelbase

        self.lateral_coefficients = self.plan_lane_change_polynomial()
        self.longitudinal_coefficients = np.polynomial.Polynomial([self.initial_state[1], self.initial_state[3]])

        self.t = 0.0

    def plan_lane_change_polynomial(self):
        t = self.lane_change_time
        b = np.array([
            [0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 2, 0, 0],
            [t ** 5, t ** 4, t ** 3, t ** 2, t, 1],
            [5 * t ** 4, 4 * t ** 3, 3 * t ** 2, 2 * t, 1, 0],
            [20 * t ** 3, 12 * t ** 2, 6 * t, 2, 0, 0]
        ])

        results = np.array([
            self.initial_state[2],
            0,
            0,
            self.initial_state[2] + self.target_y,
            0,
            0
        ])
        inverse_b = np.linalg.inv(b)
        polynomial_coefficients = np.matmul(inverse_b, results)
        polynomial = np.polynomial.Polynomial(polynomial_coefficients[::-1])
        return polynomial

    def update(self, sampling_time):
        self.t += sampling_time
        curvature = self.calculate_curvature()
        steering_angle = self.calculate_steering_angle(curvature)
        return steering_angle

    def calculate_curvature(self):
        xd = self.longitudinal_coefficients.deriv(1)(self.t)
        xdd = self.longitudinal_coefficients.deriv(2)(self.t)

        yd = self.lateral_coefficients.deriv(1)(self.t)
        ydd = self.lateral_coefficients.deriv(2)(self.t)
        return (xd * ydd - yd * xdd) / (xd ** 2 + yd ** 2) ** 1.5

    def calculate_steering_angle(self, curvature):
        return np.arctan(self.wheelbase * curvature)

    def is_end(self):
        return self.t >= self.lane_change_time


def main():
    lane_change_time = 5.0
    initial_y = 0.0
    initial_vx = 10.0
    initial_state = np.array([1, 0.0, initial_y, initial_vx, 0.0, 0.0])
    target_y = 4.0
    wheelbase = 5.0

    lane_change_trajectory_planner = LaneChangeTrajectoryPlanner(lane_change_time, initial_state, target_y, wheelbase)

    times = np.arange(0, lane_change_time + 0.1, 0.1)
    target_xs = [lane_change_trajectory_planner.longitudinal_coefficients(time) for time in times]

    target_ys = [np.matmul(
        np.array([time ** 5, time ** 4, time ** 3, time ** 2, time, 1]),
        lane_change_trajectory_planner.lateral_coefficients.coef[::-1]
    ) for time in times]
    # target_ys = [lane_change_trajectory_planner.lateral_coefficients(time) for time in times]
    target_yds = [lane_change_trajectory_planner.lateral_coefficients.deriv(1)(time) for time in times]
    target_ydds = [lane_change_trajectory_planner.lateral_coefficients.deriv(2)(time) for time in times]

    import matplotlib.pyplot as plt

    fig = plt.figure(1)
    ax = fig.add_subplot(3, 1, 1)
    ax.plot(times, target_ys)
    ax.grid(True)
    ax.set(xlim=[times[0], times[-1]], ylabel='Y [m]')

    ax = fig.add_subplot(3, 1, 2)
    ax.plot(times, target_yds)
    ax.grid(True)
    ax.set(xlim=[times[0], times[-1]], ylabel='Y Velocity [m/s]')

    ax = fig.add_subplot(3, 1, 3)
    ax.plot(times, target_ydds)
    ax.grid(True)
    ax.set(xlim=[times[0], times[-1]], xlabel='Time [s]', ylabel='Y Acceleration [m/s^2]')

    fig = plt.figure(2)
    plt.plot(target_xs, target_ys)
    plt.grid(True)
    plt.axis("equal")
    plt.xlim([target_xs[0], target_xs[-1]])
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Lane Change Path")
    plt.show()


if __name__ == "__main__":
    main()
