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
            DeclareLaunchArgument(
                name="town", default_value="Town01"
            ),  # TODO: Change this to HD_opt10 or whatever
            DeclareLaunchArgument(name="sun_azimuth", default_value="60.0"),
            DeclareLaunchArgument(name="sun_elevation", default_value="5.0"),
            DeclareLaunchArgument(name="pedestrian_number", default_value="2"),
            DeclareLaunchArgument(
                name="spawn_point", default_value="338.7,-290.0,2.0,0,0,90"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("carla_ad_demo"),
                        "carla_ad_demo.launch.py",
                    )
                ),
                launch_arguments={
                    "role_name": "ego_vehicle",
                    "spawn_point": LaunchConfiguration("spawn_point"),
                    "avoid_risk": "True",
                    "synchronous_mode_wait_for_vehicle_control_command": "True",
                }.items(),
            ),
            Node(
                package="training_scenario",
                namespace="training_scenario1",
                executable="training_scenario",
                output="screen",
                name="train",
                parameters=[
                    {"sun_azimuth": LaunchConfiguration("sun_azimuth")},
                    {"sun_elevation": LaunchConfiguration("sun_elevation")},
                    {"pedestrian_number": LaunchConfiguration("pedestrian_number")},
                ],
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
