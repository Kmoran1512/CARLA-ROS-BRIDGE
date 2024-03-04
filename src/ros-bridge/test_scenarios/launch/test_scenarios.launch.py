import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    with_driver_test = False

    descriptions = [
        # Weather Information
        DeclareLaunchArgument(name="sun_azimuth", default_value="60.0"),
        DeclareLaunchArgument(name="sun_elevation", default_value="5.0"),
        # Spawn Settings
        DeclareLaunchArgument(name="ego_side", default_value="right"),
        DeclareLaunchArgument(name="scenario_number", default_value="3"),
        DeclareLaunchArgument(name="avoid_pedestrian", default_value="False"),
        # Driving Settings
        DeclareLaunchArgument(name="target_speed", default_value="12.0"),
        DeclareLaunchArgument(name="goal", default_value="0.0,0.0,0.0,0,0,90"),
        # Recorder Settings
        DeclareLaunchArgument(name="record_gaze", default_value="False"),
        DeclareLaunchArgument(name="draw_gaze", default_value="False"),
        DeclareLaunchArgument(name="draw_outline", default_value="False"),
        # Additional Nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("carla_ad_demo"),
                    "carla_ad_demo.launch.py",
                )
            ),
            launch_arguments={
                "role_name": "ego_vehicle",
                "avoid_pedestrian": LaunchConfiguration("avoid_pedestrian"),
                "synchronous_mode_wait_for_vehicle_control_command": "True",
                "target_speed": LaunchConfiguration("target_speed"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ros_g29_force_feedback"),
                    "launch",
                    "g29_feedback.launch.py",
                )
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("record_node"), "record_node.launch.py"
                )
            ),
            launch_arguments={
                "record_gaze": LaunchConfiguration("record_gaze"),
                "draw_gaze": LaunchConfiguration("draw_gaze"),
                "draw_outline": LaunchConfiguration("draw_outline"),
                "scenario_number": LaunchConfiguration("scenario_number"),
            }.items(),
        ),
        # Run Node
        Node(
            package="image_view",
            executable="image_view",
            remappings=[("image", "/debug_img_view")],
        ),
        Node(
            package="test_scenarios",
            namespace="test_scenarios1",
            executable="test_scenarios",
            output="screen",
            name="test",
            parameters=[
                {"sun_azimuth": LaunchConfiguration("sun_azimuth")},
                {"sun_elevation": LaunchConfiguration("sun_elevation")},
                {"ego_side": LaunchConfiguration("ego_side")},
                {"scenario_number": LaunchConfiguration("scenario_number")},
                {"avoid_pedestrian": LaunchConfiguration("avoid_pedestrian")},
            ],
        ),
    ]

    if with_driver_test:
        descriptions.append(
            Node(
                package="image_view",
                executable="image_view",
                remappings=[("image", "/driver_img_view")],
            )
        )

    return LaunchDescription(descriptions)


if __name__ == "__main__":
    generate_launch_description()
