"""
Path Tracking Control Main
"""

import math

import gymnasium as gym
import numpy as np

from highway_env import utils
from src.lateral_control.linear_quadratic_gaussian import LinearQuadraticGaussian
from src.lateral_control.linear_quadratic_regulator import LinearQuadraticRegulator
from src.lateral_control.model_predictive_control import ModelPredictiveControl
from src.lateral_control.pure_pursuit import PurePursuit
from src.lateral_control.speed_controller import SpeedController
from src.lateral_control.stanley_method import StanleyMethod


def main():
    env = gym.make("lateral-control-racetrack-v0", render_mode="rgb_array")
    # Env 설정
    env.unwrapped.configure(
        {
            # Simulation Time 설정
            "duration": 40,  # [s]
            "simulation_frequency": 50,  # [Hz]
            "policy_frequency": 50,  # [Hz]

            "real_time_rendering": False,

            # Control 설정
            "action": {
                "type": "ContinuousAction",
                "longitudinal": True,
                "lateral": True,
                "acceleration_range": [-9.81, 9.81],
                "steering_range": [-np.radians(40), np.radians(40)],
                "dynamical": True,
            },

            # Observation 설정
            "observation": {
                "type": "Kinematics",
                "vehicles_count": 1,
                "features": ["x", "y", "vx", "vy", "heading", "lat_off", "ang_off"],
                "normalize": False,
                "absolute": True
            },

            "init_lateral_bias": 0.0,
        }
    )
    obs, info = env.reset()

    obs_log = obs
    action_log = np.array(0)

    sampling_time = 1 / env.unwrapped.config["policy_frequency"]
    simulation_steps = env.unwrapped.config["duration"] * env.unwrapped.config["policy_frequency"]

    ranges = {
        "acceleration_range": env.unwrapped.config["action"]["acceleration_range"],
        "steering_range": env.unwrapped.config["action"]["steering_range"],
    }
    steering_rate = np.radians(200)

    control_modes = ["pure_pursuit", "stanley", "lqr", "lqg", "mpc"]
    control_mode = control_modes[3]
    if control_mode == "pure_pursuit":
        steering_controller = PurePursuit(env.unwrapped, gain=0.5, min_distance=1)
    elif control_mode == "stanley":
        steering_controller = StanleyMethod(env.unwrapped, gain=2.0)
    elif control_mode == "lqr":
        steering_controller = LinearQuadraticRegulator(env.unwrapped, Q=np.diag([10, 1, 10, 1]), R=np.diag([5]),
                                                       dt=sampling_time)
    elif control_mode == "lqg":
        steering_controller = LinearQuadraticGaussian(env.unwrapped, Q=np.diag([10, 1, 10, 1]), R=np.diag([1]),
                                                      W=np.diag([1e-6, 1e-6, 1e-6, 1e-6]), V=np.diag([1e-6, 0.1]),
                                                      dt=sampling_time, has_noise=True)
    elif control_mode == "mpc":
        steering_controller = ModelPredictiveControl(env.unwrapped, R=np.diag([10]), Rd=np.diag([1]),
                                                     Q=np.diag([0.0, 0.5, 0.5, 100.0, 100.0]), time_horizon=10,
                                                     steering_constraint=ranges["steering_range"][1],
                                                     steering_rate_constraint=steering_rate,
                                                     dt=sampling_time)

    speed_controller = SpeedController(env.unwrapped, gain=0.5)

    previous_steering = 0.0
    for step in range(simulation_steps):
        # Autonomous Driving Logic
        steering = steering_controller.update(obs[0])
        steering = rate_limit(steering, previous_steering, steering_rate * sampling_time)
        previous_steering = steering

        acceleration = speed_controller.update(obs[0])

        action = map_action_to_interval([acceleration, steering], ranges)
        # Update Step
        obs, reward, done, truncated, info = env.step(action)
        env.render()

        obs_log = np.append(obs_log, obs, axis=0)
        action_log = np.append(action_log, steering)

    plot_simulation_result(simulation_steps, sampling_time, obs_log, action_log)


def map_action_to_interval(action, ranges):
    action[0] = utils.lmap(action[0], ranges["acceleration_range"], [-1, 1])
    action[1] = utils.lmap(action[1], ranges["steering_range"], [-1, 1])
    return action


def rate_limit(action, previous_action, limit):
    action = np.clip(action, previous_action - limit, previous_action + limit)
    return action


def plot_simulation_result(simulation_steps, sampling_time, obs_log, action_log):
    import matplotlib.pyplot as plt

    simulation_times = np.arange(0, simulation_steps * sampling_time, sampling_time)

    # Lateral Offset, Heading Offset
    fig = plt.figure(1)
    ax = fig.add_subplot(2, 1, 1)
    ax.plot(simulation_times, obs_log[1:, 5])
    ax.grid(True)
    ax.set(xlim=[simulation_times[0], simulation_times[-1]], ylim=[-1, 1], ylabel='Lateral Offset [m]')

    ax = fig.add_subplot(2, 1, 2)
    ax.plot(simulation_times, [math.degrees(angle) for angle in obs_log[1:, 6]])
    ax.grid(True)
    ax.set(xlim=[simulation_times[0], simulation_times[-1]], ylim=[-30, 30], xlabel='Time [s]',
           ylabel='Heading Offset [deg]')

    print(f"lateral offset error mean: {np.average(np.abs(obs_log[1:, 5]))}")
    print(f"heading offset error mean: {np.average(np.abs([math.degrees(angle) for angle in obs_log[1:, 6]]))}")

    # Speed, Steering Angle
    fig = plt.figure(2)
    ax = fig.add_subplot(2, 1, 1)
    ax.plot(simulation_times, [math.sqrt(v[0] ** 2 + v[1] ** 2) for v in obs_log[1:, 2:4]])
    ax.grid(True)
    ax.set(xlim=[simulation_times[0], simulation_times[-1]], ylim=[0, 15], ylabel='Speed [m/s]')

    ax = fig.add_subplot(2, 1, 2)
    ax.plot(simulation_times, [math.degrees(angle) for angle in action_log[1:]])
    ax.grid(True)
    ax.set(xlim=[simulation_times[0], simulation_times[-1]], ylim=[-45, 45], xlabel='Time [s]',
           ylabel='Steering Angle [deg]')

    plt.show()


if __name__ == "__main__":
    main()
