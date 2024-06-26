import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(name="record_gaze", default_value="False"),
        DeclareLaunchArgument(name="scenario_number", default_value="1"),
        DeclareLaunchArgument(name="draw_gaze", default_value="False"),
        DeclareLaunchArgument(name="participant_number", default_value="0"),
        # Run Node
        Node(
            package="record_node",
            namespace="record_node1",
            executable="record_node",
            output="screen",
            name="test",
            parameters=[
                {"record_gaze": LaunchConfiguration("record_gaze")},
                {"scenario_number": LaunchConfiguration("scenario_number")},
                {"participant_number": LaunchConfiguration("participant_number")},
                {"draw_gaze": LaunchConfiguration("draw_gaze")},
            ],
        ),
        Node(
            package="record_node", executable="bag_remap", output="screen", name="test"
        ),
    ]

    ld = LaunchDescription(launch_args)

    return ld


if __name__ == "__main__":
    generate_launch_description()
