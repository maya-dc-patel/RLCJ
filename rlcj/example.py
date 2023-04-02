import rlcj
import gymnasium as gym
from stable_baselines3 import PPO

env = gym.make("rlcj")
model = PPO("MlpPolicy", env)
model.learn(total_timesteps=10000000)
print("Done")
