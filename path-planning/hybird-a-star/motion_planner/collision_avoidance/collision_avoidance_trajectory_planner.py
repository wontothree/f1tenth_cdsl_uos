"""
Motion Planner Collision Avoidance
"""

import cvxpy as cp
import numpy as np


class CollisionAvoidanceTrajectoryPlanner:
    lane_width: float = 4.0
    vehicle_length: float = 5.0
    vehicle_width: float = 2.0

    def __init__(self, target_vx, headway_time, Q, R, ax_constraints, ay_constraints, time_horizon, dt):
        self.x_number = 4
        self.u_number = 2

        self.target_vx = target_vx
        self.headway_time = headway_time
        self.Q = Q
        self.R = R
        self.S = np.diag([0, 0, 50000])
        self.ax_constraints = ax_constraints
        self.ay_constraints = ay_constraints

        self.time_horizon = time_horizon
        self.dt = dt

        self.ego_vehicle: np.ndarray = np.array([])
        self.ego_current_lane: int = 0
        self.front_vehicle: np.ndarray = np.array([])

        self.trajectory_x = [0]
        self.trajectory_y = [0]
        self.axs = [0]
        self.ays = [0]

    def update(self, obs):
        self._update_ego_vehicle(obs)
        self._update_current_front_vehicle(obs)
        self._plan_optimal_trajectory()
        return self.trajectory_x, self.trajectory_y, self.axs, self.ays

    def _update_ego_vehicle(self, obs):
        self.ego_vehicle = obs[0]
        self.ego_current_lane = self.find_current_lane(self.ego_vehicle)

    def _update_current_front_vehicle(self, obs):
        front_vehicles = [vehicle for vehicle in obs[1:] if vehicle[0] == 1 and vehicle[1] >= self.ego_vehicle[1]]

        current_front_vehicles = [vehicle for vehicle in front_vehicles if
                                  self.find_current_lane(vehicle) == self.ego_current_lane]

        self.front_vehicle = self.find_closest_vehicle(current_front_vehicles)

    def find_current_lane(self, vehicle) -> int:
        return int((vehicle[2] + self.lane_width / 2) // self.lane_width) + 1

    def find_closest_vehicle(self, vehicles: list) -> np.ndarray:
        if not vehicles:
            return np.array([1, 500 + self.ego_vehicle[1], self.ego_vehicle[2], 0, 0, 0])

        return min(vehicles, key=lambda vehicle: abs(vehicle[1] - self.ego_vehicle[1]))

    def _plan_optimal_trajectory(self):
        x0 = self._get_state()

        x = cp.Variable((self.x_number, self.time_horizon + 1))
        u = cp.Variable((self.u_number, self.time_horizon))
        # slack variable: xjf, yj, if
        epsilon = cp.Variable((3, self.time_horizon))

        xref = np.zeros((self.x_number, self.time_horizon + 1))
        xref[1, :] = (self.ego_current_lane - 1) * self.lane_width
        xref[2, :] = self.target_vx

        # constant parameters
        lf = self.ego_vehicle[3] * self.headway_time + self.vehicle_length
        w = self.lane_width / 2 + self.vehicle_width
        nu = -x0[0]
        sigma = 4.5
        psi = 7
        phi = max(psi, abs(x0[0]))

        cost = 0.0
        constraints = []
        for t in range(self.time_horizon):
            cost += cp.quad_form(xref[:, t + 1] - x[:, t + 1], self.Q)
            cost += cp.quad_form(u[:, t], self.R)
            cost += cp.quad_form(epsilon[:, t], self.S)

            A, B, C = self._get_dynamics_model()
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + (C @ [self.front_vehicle[3]]).A1]
            sign = -1 if self.ego_current_lane == 1 else 1
            constraints += [x[0, t] / lf + sign * (self.front_vehicle[2] - x[1, t]) / w +
                            nu * epsilon[0, t] + epsilon[1, t] / phi + epsilon[2, t] >= 1]
            constraints += [epsilon[1, t] == -self.front_vehicle[2] + x[1, t] - sigma]

        constraints += [x[:, 0] == x0]
        constraints += [u[0, :] <= self.ax_constraints[1]]
        constraints += [u[0, :] >= self.ax_constraints[0]]
        constraints += [u[1, :] <= self.ay_constraints[1]]
        constraints += [u[1, :] >= self.ay_constraints[0]]
        constraints += [x[3, :] >= - 0.17 * x[2, :]]
        constraints += [x[3, :] <= 0.17 * x[2, :]]
        constraints += [x[1, :] <= self.lane_width * 1.5 - self.vehicle_width / 2]
        constraints += [x[1, :] >= - self.lane_width * 0.5 + self.vehicle_width / 2]

        constraints += [epsilon[0, :] >= 0]
        constraints += [epsilon[2, :] >= 0]

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.ECOS, verbose=False)

        if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
            self.trajectory_x = -np.array(x.value[0, :]).flatten() + self.front_vehicle[1] - self.vehicle_length
            self.trajectory_y = np.array(x.value[1, :]).flatten()
            self.axs = np.array(u.value[0, :]).flatten()
            self.ays = np.array(u.value[1, :]).flatten()

        else:
            print("Fail to find optimal solution.")
            self.trajectory_x = [self.ego_vehicle[1]]
            self.trajectory_y = [self.ego_vehicle[2]]
            self.axs = [0]
            self.ays = [0]

    def _get_state(self):
        relative_distance = self.front_vehicle[1] - self.ego_vehicle[1] - self.vehicle_length

        return [relative_distance, self.ego_vehicle[2], self.ego_vehicle[3], self.ego_vehicle[4]]

    def _get_dynamics_model(self):
        A_c = np.matrix([
            [0, 0, -1, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ])

        B_c = np.matrix([
            [0, 0],
            [0, 0],
            [1, 0],
            [0, 1]
        ])

        C_c = np.matrix([
            [1],
            [0],
            [0],
            [0]
        ])

        A = np.eye(self.x_number) + self.dt * A_c
        B = self.dt * B_c
        C = self.dt * C_c

        return A, B, C


def main():
    time_horizon = 40
    dt = 0.1
    collision_avoidance_planner = CollisionAvoidanceTrajectoryPlanner(
        target_vx=15,
        headway_time=1,
        Q=np.diag([0, 50, 50, 50]),
        R=np.diag([100, 200]),
        ax_constraints=[-4, 1],
        ay_constraints=[-2, 2],
        time_horizon=time_horizon,
        dt=dt,
    )

    obs = np.array([
        [1, 0, 0, 15, 0, 0],
        [1, 40, 0, 0, 0, 0]
    ])
    xs, ys, axs, ays = collision_avoidance_planner.update(obs)

    import matplotlib.pyplot as plt
    times = np.arange(0, time_horizon * dt, dt) if len(xs) != 1 else [0.0]
    fig = plt.figure(1)
    ax = fig.add_subplot(2, 1, 1)
    ax.plot(times, axs)
    ax.grid(True)
    ax.set(xlim=[times[0], times[-1]], ylim=[-4, 4], ylabel='ax [m/s^2]')

    ax = fig.add_subplot(2, 1, 2)
    ax.plot(times, ays)
    ax.grid(True)
    ax.set(xlim=[times[0], times[-1]], ylim=[-4, 4], xlabel='Time [s]', ylabel='ay [m/s^2]')

    fig = plt.figure(2)
    plt.plot(xs, ys, "b-o", label="trajectory")
    plt.plot(obs[0, 1], obs[0, 2], "yo", label="Ego Vehicle")
    plt.plot(obs[1, 1], obs[1, 2], "ro", label="Front Vehicle")
    plt.plot([obs[0, 1] - 10, obs[1, 1] + 10], [-2, - 2], "k", label="Constraint")
    plt.plot([obs[0, 1] - 10, obs[1, 1] + 10], [2, 2], "k--")
    plt.plot([obs[0, 1] - 10, obs[1, 1] + 10], [6, 6], "k")

    lf = obs[0, 3] * collision_avoidance_planner.headway_time + collision_avoidance_planner.vehicle_length
    w = collision_avoidance_planner.lane_width / 2 + collision_avoidance_planner.vehicle_width
    sign = -1 if -2 <= obs[1, 2] <= 2 else 1
    plt.plot([obs[1, 1] + sign * (obs[1, 2] - 6) * lf / w - lf, obs[1, 1] + sign * (obs[1, 2] + 2) * lf / w - lf],
             [6, -2], "k")

    plt.plot
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.title("Planning Result")
    plt.show()


if __name__ == "__main__":
    main()
