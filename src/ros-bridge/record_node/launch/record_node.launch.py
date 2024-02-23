import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription(
        [
            # Run Node
            Node(
                package="record_node",
                namespace="record_node1",
                executable="record_node",
                output="screen",
                name="test",
            )
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
