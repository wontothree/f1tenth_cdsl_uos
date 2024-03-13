"""
Behavior Decision-Making Main (Practice)
"""

import gymnasium as gym
import numpy as np

from src.decision_making.behavior_decision_making_practice import BehaviorDecisionMaking


def main():
    env = gym.make("decision-making-highway-v0", render_mode="rgb_array")
    # Env 설정
    env.unwrapped.configure(
        {
            # Simulation Time 설정
            "duration": 20,  # [s]
            "simulation_frequency": 50,  # [Hz]
            "policy_frequency": 50,  # [Hz]

            "real_time_rendering": True,

            "unsafe_scenario": True,

            # Control 설정
            "action": {
                "type": "DiscreteMetaAction",
            },

            # Observation 설정
            "observation": {
                "type": "Kinematics",
                "vehicles_count": 11,
                "features": ["presence", "x", "y", "vx", "vy"],
                "normalize": False,
                "absolute": True,
                "see_behind": True,
                "order": "sorted",
            },
        }
    )
    obs, info = env.reset()

    # 제어 하는 차량의 Target Speed 를 설정 (min / max)
    env.unwrapped.controlled_vehicles[0].target_speeds = np.array([10, 20])

    sampling_time = 1 / env.unwrapped.config["policy_frequency"]
    simulation_steps = env.unwrapped.config["duration"] * env.unwrapped.config["policy_frequency"]

    behavior_decision_making = BehaviorDecisionMaking(sampling_time)

    for step in range(simulation_steps):
        # Autonomous Driving Logic
        action = behavior_decision_making.update(obs, step)

        # Update Step
        obs, reward, done, truncated, info = env.step(action)
        env.render()


if __name__ == "__main__":
    main()
