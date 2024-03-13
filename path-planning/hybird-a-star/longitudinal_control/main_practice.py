"""
Longitudinal Control (Practice)
"""

import gymnasium as gym
import numpy as np

from highway_env import utils
from src.longitudinal_control.linear_quadratic_regulator_practice import LinearQuadraticRegulator
from src.longitudinal_control.linear_quadratic_regulator_with_delay_practice import LinearQuadraticRegulatorWithDelay
from src.longitudinal_control.model_predictive_control_practice import ModelPredictiveControl
from src.longitudinal_control.pid_controller_practice import PidController


def main():
    env = gym.make("longitudinal-control-highway-v0", render_mode="rgb_array")
    # Env 설정
    env.unwrapped.configure(
        {
            # Simulation Time 설정
            "duration": 10,  # [s]
            "simulation_frequency": 50,  # [Hz]
            "policy_frequency": 50,  # [Hz]

            "real_time_rendering": True,

            # Control 설정
            "action": {
                "type": "ContinuousAction",
                "longitudinal": True,
                "lateral": True,
                "acceleration_range": [-9.81, 9.81],
                "steering_range": [-np.radians(40), np.radians(40)],
            },

            # Observation 설정
            "observation": {
                "type": "Kinematics",
                "vehicles_count": 11,
                "features": ["presence", "x", "y", "vx", "vy", "heading"],
                "normalize": False,
                "absolute": True,
                "see_behind": True,
                "order": "sorted",
            },

            "deceleration_scenario": False
        }
    )
    obs, info = env.reset()

    ego_log = obs[0]
    front_log = obs[1]
    action_log = np.array([0, 0])

    sampling_time = 1 / env.unwrapped.config["policy_frequency"]
    simulation_steps = env.unwrapped.config["duration"] * env.unwrapped.config["policy_frequency"]
    ranges = {
        "acceleration_range": env.unwrapped.config["action"]["acceleration_range"],
        "steering_range": env.unwrapped.config["action"]["steering_range"],
    }

    headway_time = 2
    control_modes = ["pid", "lqr", "lqr_delay", "mpc"]
    control_mode = control_modes[0]
    if control_mode == "pid":
        car_following_controller = PidController(headway_time=headway_time, distance_gains=[1, 0, 0],
                                                 velocity_gains=[1, 0, 0],
                                                 dt=sampling_time)
    elif control_mode == "lqr":
        car_following_controller = LinearQuadraticRegulator(headway_time=headway_time, Q=np.diag([1, 1]),
                                                            R=np.diag([1]),
                                                            dt=sampling_time)
    elif control_mode == "lqr_delay":
        car_following_controller = LinearQuadraticRegulatorWithDelay(headway_time=headway_time,
                                                                     Q=np.diag([1, 1, 1]),
                                                                     R=np.diag([1]), dt=sampling_time)
    elif control_mode == "mpc":
        car_following_controller = ModelPredictiveControl(headway_time=headway_time, Q=np.diag([1, 1, 1]),
                                                          R=np.diag([1]), Rd=np.diag([1]), time_horizon=10,
                                                          dt=sampling_time)

    vehicle_acceleration = 0.0
    for step in range(simulation_steps):
        # Autonomous Driving Logic
        acceleration_command = car_following_controller.update(obs, vehicle_acceleration)
        vehicle_acceleration = first_order_delay(acceleration_command, vehicle_acceleration, 0.45, sampling_time)
        action = [vehicle_acceleration, 0.0]
        # Update Step
        obs, reward, done, truncated, info = env.step(map_action_to_interval(action, ranges))
        env.render()

        ego_log = np.vstack([ego_log, car_following_controller.ego_vehicle])
        front_log = np.vstack([front_log, car_following_controller.front_vehicle])
        action_log = np.vstack([action_log, np.array([acceleration_command, vehicle_acceleration])])

    plot_simulation_result(simulation_steps, sampling_time, ego_log, front_log, action_log, headway_time)


def map_action_to_interval(action, ranges):
    action[0] = utils.lmap(action[0], ranges["acceleration_range"], [-1, 1])
    action[1] = utils.lmap(action[1], ranges["steering_range"], [-1, 1])
    return action


def first_order_delay(x, prev_x, time_constant, dt):
    return (1 - dt / time_constant) * prev_x + dt / time_constant * x


def plot_simulation_result(simulation_steps, sampling_time, ego_log, front_log, action_log, headway_time):
    import matplotlib.pyplot as plt

    simulation_times = np.arange(0, simulation_steps * sampling_time, sampling_time)

    # Acceleration
    fig = plt.figure(1)
    ax = fig.add_subplot(2, 1, 1)
    ax.plot(simulation_times, action_log[1:, 0], label="Command")
    ax.plot(simulation_times, action_log[1:, 1], label="Delayed Command")
    ax.grid(True)
    ax.legend()
    ax.set(xlim=[simulation_times[0], simulation_times[-1]], ylim=[-10, 10], ylabel='acceleration [m/s^2]')

    # Relative Distance, Relative Velocity
    fig = plt.figure(2)
    ax = fig.add_subplot(2, 1, 1)
    ax.plot(simulation_times, 2 * ego_log[1:, 3], label='Target Distance')
    ax.plot(simulation_times, front_log[1:, 1] - ego_log[1:, 1] - 5, label='Relative Distance')
    ax.grid(True)
    ax.legend()
    ax.set(xlim=[simulation_times[0], simulation_times[-1]], ylim=[0, 50], ylabel='Relative Distance [m]')

    ax = fig.add_subplot(2, 1, 2)
    ax.plot(simulation_times, headway_time * ego_log[1:, 3] - front_log[1:, 1] + ego_log[1:, 1] + 5)
    ax.grid(True)
    ax.set(xlim=[simulation_times[0], simulation_times[-1]], ylim=[-10, 10], xlabel='Time [s]',
           ylabel='Distance Error [m]')

    print(
        f"distance error mean: {np.average(np.abs(headway_time * ego_log[1:, 3] - front_log[1:, 1] + ego_log[1:, 1] + 5))}"
        f" / max: {np.max(np.abs(headway_time * ego_log[1:, 3] - front_log[1:, 1] + ego_log[1:, 1] + 5))}")

    # Relative Distance, Relative Velocity
    fig = plt.figure(3)
    ax = fig.add_subplot(2, 1, 1)
    ax.plot(simulation_times, front_log[1:, 3], label='Target Velocity')
    ax.plot(simulation_times, ego_log[1:, 3], label='Ego Velocity')
    ax.grid(True)
    ax.legend()
    ax.set(xlim=[simulation_times[0], simulation_times[-1]], ylim=[0, 25], ylabel='Velocity [m/s]')

    ax = fig.add_subplot(2, 1, 2)
    ax.plot(simulation_times, front_log[1:, 3] - ego_log[1:, 3])
    ax.grid(True)
    ax.set(xlim=[simulation_times[0], simulation_times[-1]], ylim=[-5, 5], xlabel='Time [s]',
           ylabel='Velocity Error [m/s]')

    print(f"velocity error mean: {np.average(np.abs(front_log[1:, 3] - ego_log[1:, 3]))}"
          f" / max: {np.max(np.abs(front_log[1:, 3] - ego_log[1:, 3]))}")

    plt.show()


if __name__ == "__main__":
    main()
