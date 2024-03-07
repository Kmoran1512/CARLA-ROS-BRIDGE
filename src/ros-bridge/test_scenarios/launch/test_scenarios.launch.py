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
    sun_elevation = 5.0

    descriptions = [
        # Spawn Settings
        DeclareLaunchArgument(name="peds", default_value="[0,1,0]"),
        DeclareLaunchArgument(name="bp", default_value="1"),
        DeclareLaunchArgument(name="bps", default_value=""),
        DeclareLaunchArgument(name="direction", default_value="0.0"),
        DeclareLaunchArgument(name="directions", default_value=""),
        DeclareLaunchArgument(name="speed", default_value="0.0"),
        DeclareLaunchArgument(name="speeds", default_value=""),
        DeclareLaunchArgument(name="tdelay", default_value="0.0"),
        DeclareLaunchArgument(name="tdelays", default_value=""),
        DeclareLaunchArgument(name="mdelay", default_value="-1.0"),
        DeclareLaunchArgument(name="mdelays", default_value=""),
        # Driving Settings
        DeclareLaunchArgument(name="target_speed", default_value="12.0"),
        DeclareLaunchArgument(name="avoid_pedestrian", default_value="False"),
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
                {"peds": LaunchConfiguration("peds")},
                {"bp": LaunchConfiguration("bp")},
                {"bps": LaunchConfiguration("bps")},
                {"direction": LaunchConfiguration("direction")},
                {"directions": LaunchConfiguration("directions")},
                {"speed": LaunchConfiguration("speed")},
                {"speeds": LaunchConfiguration("speeds")},
                {"tdelay": LaunchConfiguration("tdelay")},
                {"tdelays": LaunchConfiguration("tdelays")},
                {"mdelay": LaunchConfiguration("mdelay")},
                {"mdelays": LaunchConfiguration("mdelays")},
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
