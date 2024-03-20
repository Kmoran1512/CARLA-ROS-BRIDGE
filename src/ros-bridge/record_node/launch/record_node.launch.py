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
            DeclareLaunchArgument(name="record_gaze", default_value="False"),
            DeclareLaunchArgument(name="participant_number", default_value="1"),
            DeclareLaunchArgument(name="scenario_number", default_value="1"),
            DeclareLaunchArgument(name="draw_manctrl", default_value="True"),
            DeclareLaunchArgument(name="draw_gaze", default_value="False"),
            DeclareLaunchArgument(name="draw_outline", default_value="False"),
            DeclareLaunchArgument(name="draw_route", default_value="False"),
            DeclareLaunchArgument(name="labels", default_value="[]]"),
            # Run Node
            Node(
                package="record_node",
                namespace="img_publisher1",
                executable="img_publisher",
                output="screen",
                name="test",
                parameters=[
                    {"draw_manctrl": LaunchConfiguration("draw_manctrl")},
                    {"draw_gaze": LaunchConfiguration("draw_gaze")},
                    {"draw_outline": LaunchConfiguration("draw_outline")},
                    {"draw_route": LaunchConfiguration("draw_route")},
                    {"labels": LaunchConfiguration("labels")},
                ],
            ),
            Node(
                package="record_node",
                namespace="record_node1",
                executable="record_node",
                output="screen",
                name="test",
                parameters=[
                    {"record_gaze": LaunchConfiguration("record_gaze")},
                    {"participant_number": LaunchConfiguration("participant_number")},
                    {"scenario_number": LaunchConfiguration("scenario_number")},
                ],
            ),
            Node(
                package="record_node",
                namespace="semantic_boxes1",
                executable="semantic_boxes",
                output="screen",
                name="test",
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
