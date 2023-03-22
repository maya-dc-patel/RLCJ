from __future__ import annotations
import gymnasium as gym
import numpy as np
import numpy.typing as npt
import threading
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class GazeboEnv(gym.Env, Node):
    metadata = {"render_modes": ["human"]}

    def __init__(self):
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
        return obs

    def reset(self) -> tuple[npt.NDArray, None]:
        # Reset environment from main thread.
        self._reset_world.call(Empty.Request())
        with self._lock:
            self._collided = False
        return self._get_obs(), None

    def step(self, action: tuple[float, float, float]) -> tuple[npt.NDArray, int, bool, bool, None]:
        # Convert tuple to twist.
        action_msg: Twist = Twist()
        action_msg.linear.x = float(action[0])
        action_msg.linear.y = float(action[1])
        action_msg.angular.z = float(action[2])

        # Step environment from main thread.
        self._vel_cmd.publish(action_msg)
        reward: int = 1
        terminated: bool = False
        with self._lock:
            if self._collided:
                reward = -100
                terminated = True
        return self._get_obs(), reward, terminated, False, None


def main():
    ge = GazeboEnv()
    while True:
        _, _, term, _, _ = ge.step((1, 0, 0))
        print(term)
        time.sleep(1)
        if term:
            ge.reset()

if __name__ == "__main__":
    main()

