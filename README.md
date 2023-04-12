# Reinforcement Learning for the Clearpath Jackal

The goal of this project is to provide an environment to train an agent to move the [Clearpath Jackal](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/) robot without colliding into walls.

Uses [Stable-Baselines3](https://stable-baselines3.readthedocs.io/en/master/), [ROS 2](https://www.ros.org/), [Gazebo Classic](https://classic.gazebosim.org/), and the [Clearpath Jackal ROS Packages](https://github.com/jackal) within the container.

## System Requirements

[Docker](https://www.docker.com/) must be installed on your system.

## Building the Container Image

Run the following command in the repo folder.

```bash
docker build . -t rlcj
```

## Training the Agents in the Containers

The `docker-compose.yaml` file can be used to spin up the image just built. The output of the containers (logs and checkpoints) can be found in the `./output` folder by default.

```bash
docker compose up -d
```

There are four environment variables that can be used to configure a container.

* `RLCJ_ALGORITHM` - Required. Choose from `A2C`, `PPO`, and `TD3` reinforcement learning algorithms.
* `RLCJ_OUTPUT_PATH` - Optional. Defaults to `/tmp/rl_out`. Location inside of the container to output logs and checkpoint to.
* `RLCJ_CHECKPOINT_FREQUENCY` - Optional. Defaults to `1000`. How often to save a checkpoint of the model being trained..
* `RLCJ_TIMESTEPS` - Optional. Defaults to `1000000`. How many timesteps to train the agent for.
