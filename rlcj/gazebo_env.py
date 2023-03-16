from __future__ import annotations
import gymnasium as gym
import numpy as np
import numpy.typing as npt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class GazeboEnv(gym.Env, Node):
    metadata = {"render_modes": ["human"]}

    def __init__(self):
        # Initialize obervation variables.
        self.odometry: npt.ndarray = np.zeros((13,))
        self.lidar: npt.ndarray = np.zeros((720,))

        # Connect to needed ROS topics.
        Node.__init__(self, "gazebo_env")
        self.create_subscription(Odometry, "/odom", self._set_odometry, 10)
        self.create_subscription(LaserScan, "/front/scan", self._set_lidar, 10)
        self.vel_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.reset_world = self.create_client(Empty, "/reset_world")

    def _set_odometry(self, odometry: Odometry) -> None:
        self.odometry[0] = odometry.pose.pose.position.x
        self.odometry[1] = odometry.pose.pose.position.y
        self.odometry[2] = odometry.pose.pose.position.z
        self.odometry[3] = odometry.pose.pose.orientation.x
        self.odometry[4] = odometry.pose.pose.orientation.y
        self.odometry[5] = odometry.pose.pose.orientation.z
        self.odometry[6] = odometry.pose.pose.orientation.w
        self.odometry[7] = odometry.twist.twist.linear.x
        self.odometry[8] = odometry.twist.twist.linear.y
        self.odometry[9] = odometry.twist.twist.linear.z
        self.odometry[10] = odometry.twist.twist.angular.x
        self.odometry[11] = odometry.twist.twist.angular.y
        self.odometry[12] = odometry.twist.twist.angular.z

    def _set_lidar(self, lidar: LaserScan) -> None:
        self.lidar[:] = lidar.ranges

    def _get_obs(self) -> npt.NDArray:
        return np.concatenate((self.odometry, self.lidar), axis=None)

    def reset(self) -> tuple[npt.NDArray, None]:
        self.reset_world.call(Empty())
        return self._get_obs(), None

    def step(self, action: Twist) -> tuple[npt.NDArray, int, bool, bool, None]:
        self.vel_cmd.publish(action)
        reward: int = 1
        terminated: bool = False
        truncated: bool = False
        return self._get_obs(), reward, terminated, truncated, None

    def close(self):
        pass


def main():
    rclpy.init()
    rclpy.spin(GazeboEnv())
    rclpy.shutdown()
