"""
Motion Planner Parking Main (Practice)
"""

import gymnasium as gym
import numpy as np

from highway_env import utils
from src.motion_planner.parking.parking_motion_planner_practice import ParkingMotionPlanner


def main():
    env = gym.make("motion-planner-parking-v0", render_mode="rgb_array")
    # Env 설정
    env.unwrapped.configure(
        {
            # Simulation Time 설정
            "duration": 60,  # [s]
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
                "vehicles_count": 1,
                "features": ["x", "y", "vx", "vy", "heading"],
                "normalize": False,
                "absolute": True,
                "see_behind": True,
                "order": "sorted",
            },
            "parallel_scenario": False,
        }
    )
    obs, info = env.reset()

    sampling_time = 1 / env.unwrapped.config["policy_frequency"]
    simulation_steps = env.unwrapped.config["duration"] * env.unwrapped.config["policy_frequency"]
    ranges = {
        "acceleration_range": env.unwrapped.config["action"]["acceleration_range"],
        "steering_range": env.unwrapped.config["action"]["steering_range"],
    }

    parking_motion_planner = ParkingMotionPlanner(env.unwrapped.goal.position, is_front_direction=True)

    for step in range(simulation_steps):
        # Autonomous Driving Logic
        action = parking_motion_planner.update(obs[0])

        # Update Step
        obs, reward, done, truncated, info = env.step(map_action_to_interval(action, ranges))
        env.render()


def map_action_to_interval(action, ranges):
    action[0] = utils.lmap(action[0], ranges["acceleration_range"], [-1, 1])
    action[1] = utils.lmap(action[1], ranges["steering_range"], [-1, 1])
    return action


if __name__ == "__main__":
    main()
