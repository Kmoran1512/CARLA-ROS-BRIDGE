import json
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# 35 = 15.6
# 27 = 12.0
SPEED_MPH = 35
SPEED_MPS = SPEED_MPH / 2.237
LAG_FACTOR = 1.2
ADJUSTED_SPEED = SPEED_MPS * LAG_FACTOR


def generate_launch_description():
    sun_azimuth = 60.0
    sun_elevation = 15.0

    config_param = next(
        (
            config_param[8:]
            for config_param in sys.argv
            if config_param.startswith("config:=")
        ),
        "",
    ).split("-")
    location = side_map[config_param[0]]

    descriptions = [
        # Spawn Settings
        DeclareLaunchArgument(name="config", default_value=""),
        DeclareLaunchArgument(name="town", default_value="Town10HD_Opt"),
        DeclareLaunchArgument(name="spawn_point", default_value=location[0]),
        DeclareLaunchArgument(name="goal", default_value=location[1]),
        # Driving Settings
        DeclareLaunchArgument(name="target_speed", default_value=str(ADJUSTED_SPEED)),
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
            remappings=[("image", "/driver_img_view")],
        ),
        Node(
            package="test_scenarios",
            namespace="test_scenarios1",
            executable="test_scenarios",
            output="screen",
            name="test",
            parameters=[
                {"scenario_config": json.dumps(build_config(config_param))},
                {"spawn_location": location},
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

    return LaunchDescription(descriptions)


def build_config(names):
    config = {"num": 2, "pedestrians": []}

    for i, name in enumerate(names[1:]):
        config["pedestrians"].append(build_pedestrian(names[0][0], i, name))

    return config


def build_pedestrian(side, n, kind):
    LABELS = {"child": 13, "police": 30, "terrorist": 38, "bike": 101}
    RT = 2.0

    pedestrian = {"actions": [{}]}
    pedestrian["yaw"] = 90.0 if n == 0 else -90.0
    pedestrian["actions"][0]["yaw"] = 95.0 if n == 0 else -90.0

    if side == "r":
        pedestrian["actions"][0]["speed"] = 2.4 if n == 0 else 2.0
        pedestrian["spawn"] = "far_left_margin" if n == 0 else "right"
    elif side == "l":
        pedestrian["actions"][0]["speed"] = 1.7 if n == 0 else 2.0
        pedestrian["spawn"] = "near_left_margin" if n == 0 else "far_right"

    pedestrian["actions"][0]["mdelay"] = RT * ADJUSTED_SPEED
    pedestrian["blueprint"] = LABELS[kind]

    return pedestrian


side_map = {
    "ls": ("88.0, -16.6, 1.0, 0, 0, 180", "-5.5, -16.6, 0.0, 0, 0, 180"),
    "rs": ("88.0, -13.4, 1.0, 0, 0, 180", "-5.5, -13.2, 0.0, 0, 0, 180"),
    "cs": ("39.9, -66.3, 1.0, 0, 0, 180", "-41.7, -47.8, 0.0, 0, 0, 90"),
    "lt": ("105.2, 30.0, 1.0, 0, 0, 105", "30.4, 64.4, 1.0, 0, 0, 180"),
    "rt": ("109.7, 30.0, 1.0, 0, 0, 105", "30.4, 67.8, 1.0, 0, 0, 180"),
}

if __name__ == "__main__":
    generate_launch_description()
