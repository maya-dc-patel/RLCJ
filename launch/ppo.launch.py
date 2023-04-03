from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    laser = SetEnvironmentVariable(name="JACKAL_LASER", value="1")
    jackal_gazebo = get_package_share_directory("jackal_gazebo")
    jackal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{jackal_gazebo}/launch/jackal_world.launch.py")
    )
    env = Node(
        package="rlcj",
        namespace="rlcj",
        executable="ppo",
        name="ppo",
    )
    return LaunchDescription(
        [
            laser,
            jackal_launch,
            env,
        ]
    )
