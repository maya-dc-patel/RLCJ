from __future__ import annotations
import gymnasium as gym
import os
from stable_baselines3 import A2C, PPO, TD3
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure


def get_model(algorithm: str | None, env: gym.Env) -> BaseAlgorithm:
    model = None
    if algorithm is not None:
        algorithm = algorithm.upper().strip()
        if algorithm == "PPO":
            model = PPO("MlpPolicy", env)
        elif algorithm == "A2C":
            model = A2C("MlpPolicy", env)
        elif algorithm == "TD3":
            model = TD3("MlpPolicy", env)

    if model is None:
        print("RLCJ_ALGORITHM not set or invalid.")
        exit(1)
    return model


def main():
    algorithm = os.getenv("RLCJ_ALGORITHM")
    output_path = os.getenv("RLCJ_OUTPUT_PATH", "/tmp/rl_out")
    checkpoint_frequency = os.getenv("RLCJ_CHECKPOINT_FREQUENCY", 1000)
    timesteps = os.getenv("RLCJ_TIMESTEPS", 1000000)

    logger = configure(output_path, ["stdout", "log", "csv"])
    checkpoint_callback = CheckpointCallback(
        save_freq=checkpoint_frequency,
        save_path=output_path,
        name_prefix="rl_model",
        save_replay_buffer=True,
        save_vecnormalize=True,
    )

    env = make_vec_env("rlcj", n_envs=1)
    model = get_model(algorithm, env)
    model.set_logger(logger)
    model.learn(total_timesteps=timesteps, callback=checkpoint_callback)
    model.save(output_path)
