import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    with_driver_test = False

    sun_azimuth = 60.0
    sun_elevation = 15.0

    ll_c = "-110.2,17.6,2.0,0,0,270"
    # Right-Lane Curve = "-113.8,17.6,2.0,0,0,270"
    ll_s = "4.9,-16.6,2.0,0,0,180"
    rl_s = "4.9,-13.2,2.0,0,0,180"

    g_ll_c = "-37.3,-24.6,2.0,0,0,0"
    # Right-Lane Curve "-37.3,-28.2,2.0,0,0,0"
    g_ll_s = "-90.5,-16.6,2.0,0,0,180"
    g_rl_s = "-90.5,-13.2,2.0,0,0,180"

    descriptions = [
        # Spawn Settings
        DeclareLaunchArgument(name="config", default_value=""),
        DeclareLaunchArgument(name="town", default_value="Town10HD_Opt"),
        DeclareLaunchArgument(name="spawn_point", default_value=ll_c),
        DeclareLaunchArgument(name="goal", default_value=g_ll_c),
        # Driving Settings
        DeclareLaunchArgument(name="target_speed", default_value="12.0"),
        DeclareLaunchArgument(name="avoid_pedestrian", default_value="False"),
        # Recorder Settings
        DeclareLaunchArgument(name="n", default_value="1"),
        DeclareLaunchArgument(name="record_gaze", default_value="False"),
        DeclareLaunchArgument(name="draw_gaze", default_value="False"),
        DeclareLaunchArgument(name="draw_outline", default_value="False"),
        DeclareLaunchArgument(name="labels", default_value="['']"),
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
                "goal": LaunchConfiguration("goal"),
                "spawn_point": LaunchConfiguration("spawn_point"),
                "synchronous_mode_wait_for_vehicle_control_command": "True",
                "target_speed": LaunchConfiguration("target_speed"),
                "town": LaunchConfiguration("town"),
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
                "scenario_number": LaunchConfiguration("n"),
                "record_gaze": LaunchConfiguration("record_gaze"),
                "draw_gaze": LaunchConfiguration("draw_gaze"),
                "draw_outline": LaunchConfiguration("draw_outline"),
                "labels": LaunchConfiguration("labels"),
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
                {"scenario_config": LaunchConfiguration("config")},
                {"spawn_point": LaunchConfiguration("spawn_point")},
            ],
        ),
    ]

    descriptions.append(
        ExecuteProcess(
            cmd=[
                "ros2",
                "topic",
                "pub",
                "-t",
                "5",
                "/carla/weather_control",
                "carla_msgs/msg/CarlaWeatherParameters",
                f"{{sun_azimuth_angle: {sun_azimuth}, sun_altitude_angle: {sun_elevation}}}",
            ]
        )
    )

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
