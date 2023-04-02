from __future__ import annotations
from concurrent.futures import Future
import gymnasium as gym
import numpy as np
import numpy.typing as npt
import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class GazeboEnv(gym.Env, Node):
    def __init__(self):
        # Gym environment variables.
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(733,), dtype=np.float64
        )
        self.action_space = gym.spaces.Box(
            low=-2.0, high=2.0, shape=(2,), dtype=np.float32
        )

        # Initialize obervation variables.
        self._odometry: npt.ndarray = np.zeros((13,))
        self._lidar: npt.ndarray = np.zeros((720,))
        self._collided: bool = False

        # Initialize ROS and connect to needed ROS topics.
        rclpy.init()
        Node.__init__(self, "gazebo_env")
        self.create_subscription(Odometry, "/odom", self._set_odometry, 1)
        self.create_subscription(LaserScan, "/front/scan", self._set_lidar, 1)
        self.create_subscription(
            ContactsState, "/bumper_states", self._check_collision, 1
        )
        self._vel_cmd = self.create_publisher(Twist, "/cmd_vel", 1)
        self._reset_world = self.create_client(Empty, "/reset_world")

        # Node specific thread and related lock.
        self._lock: threading.Lock = threading.Lock()
        self._node: threading.Thread = threading.Thread(target=self._spin)
        self._node.start()

    def _spin(self):
        # Run ROS node indefinitely from node thread.
        rclpy.spin(self)
        rclpy.shutdown()

    def _set_odometry(self, odometry: Odometry) -> None:
        # Update odometry from node thread.
        with self._lock:
            self._odometry[0] = odometry.pose.pose.position.x
            self._odometry[1] = odometry.pose.pose.position.y
            self._odometry[2] = odometry.pose.pose.position.z
            self._odometry[3] = odometry.pose.pose.orientation.x
            self._odometry[4] = odometry.pose.pose.orientation.y
            self._odometry[5] = odometry.pose.pose.orientation.z
            self._odometry[6] = odometry.pose.pose.orientation.w
            self._odometry[7] = odometry.twist.twist.linear.x
            self._odometry[8] = odometry.twist.twist.linear.y
            self._odometry[9] = odometry.twist.twist.linear.z
            self._odometry[10] = odometry.twist.twist.angular.x
            self._odometry[11] = odometry.twist.twist.angular.y
            self._odometry[12] = odometry.twist.twist.angular.z

    def _set_lidar(self, lidar: LaserScan) -> None:
        # Update lidar from node thread.
        with self._lock:
            self._lidar[:] = lidar.ranges

    def _check_collision(self, contacts: ContactsState) -> None:
        # Check for wall collision from node thread.
        for state in contacts.states:
            if state.collision1_name[:3] == "drc" or state.collision2_name[:3] == "drc":
                with self._lock:
                    self._collided = True

    def _get_obs(self) -> npt.NDArray:
        # Retrieve observation variables from main thread.
        with self._lock:
            obs = np.concatenate((self._odometry, self._lidar), axis=None)
            obs[np.isinf(obs)] = 10  # Remove inf values and replace with max range.
        return obs

    def reset(
        self, seed: int | None = None, options: dict | None = None
    ) -> tuple[npt.NDArray, None]:
        # Reset environment from main thread.
        self._reset_world.call(Empty.Request())
        with self._lock:
            self._collided = False
        return self._get_obs(), {}

    def step(
        self, action: tuple[float, float, float]
    ) -> tuple[npt.NDArray, float, bool, bool, None]:
        # Convert tuple to twist.
        action_msg: Twist = Twist()
        action_msg.linear.x = float(action[0])
        action_msg.angular.z = float(action[1])

        # Step environment from main thread.
        self._vel_cmd.publish(action_msg)
        obs: npt.NDArray = self._get_obs()
        reward: float = np.abs(obs[7])
        terminated: bool = False
        with self._lock:
            if self._collided:
                reward = -100
                terminated = True
        return obs, reward, terminated, False, {}
