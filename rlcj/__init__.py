from gymnasium.envs.registration import register
from .gazebo_env import GazeboEnv

register(
     id="rlcj",
     entry_point="gazebo_env:GazeboEnv"
)
