"""
Motion Planner Collision Avoidance Main
"""

import gymnasium as gym
import numpy as np

from highway_env import utils
from src.motion_planner.collision_avoidance.collision_avoidance_trajectory_planner import \
    CollisionAvoidanceTrajectoryPlanner
from src.motion_planner.collision_avoidance.steering_controller import SteeringController


def main():
    env = gym.make("motion-planner-collision-avoidance-v0", render_mode="rgb_array")
    # Env 설정
    env.unwrapped.configure(
        {
            # Simulation Time 설정
            "duration": 25,  # [s]
            "simulation_frequency": 50,  # [Hz]
            "policy_frequency": 10,  # [Hz]

            "real_time_rendering": False,

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
                "vehicles_count": 4,
                "features": ["presence", "x", "y", "vx", "vy", "heading"],
                "normalize": False,
                "absolute": True,
                "see_behind": True,
                "order": "sorted",
            },
        }
    )
    obs, info = env.reset()

    sampling_time = 1 / env.unwrapped.config["policy_frequency"]
    simulation_steps = env.unwrapped.config["duration"] * env.unwrapped.config["policy_frequency"]
    ranges = {
        "acceleration_range": env.unwrapped.config["action"]["acceleration_range"],
        "steering_range": env.unwrapped.config["action"]["steering_range"],
    }

    time_horizon = 20
    collision_avoidance_trajectory_planner = CollisionAvoidanceTrajectoryPlanner(
        target_vx=15,
        headway_time=1.5,
        Q=np.diag([0, 200, 50, 10]),
        R=np.diag([100, 10]),
        ax_constraints=[-4, 1],
        ay_constraints=[-2, 2],
        time_horizon=time_horizon,
        dt=sampling_time,
    )
    steering_control = SteeringController(gain=1, min_distance=5)

    for step in range(simulation_steps):
        # Autonomous Driving Logic
        xs, ys, axs, ays = collision_avoidance_trajectory_planner.update(obs)

        # Trajectory Tracking 위한 Control
        steering = steering_control.update(obs[0], xs, ys)
        action = [axs[0], steering]

        # Update Step
        obs, reward, done, truncated, info = env.step(map_action_to_interval(action, ranges))
        env.render()


def map_action_to_interval(action, ranges):
    action[0] = utils.lmap(action[0], ranges["acceleration_range"], [-1, 1])
    action[1] = utils.lmap(action[1], ranges["steering_range"], [-1, 1])
    return action


if __name__ == "__main__":
    main()
