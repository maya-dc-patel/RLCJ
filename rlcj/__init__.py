from gymnasium.envs.registration import register
from rlcj.gazebo_env import GazeboEnv

register(
     id="rlcj",
     entry_point="rlcj.gazebo_env:GazeboEnv"
)
