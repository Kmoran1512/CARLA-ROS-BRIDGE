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
from test_scenarios.spawn_helper import build_pedestrian, get_pedestrian_labels

# 35 = 15.6
# 27 = 12.0
SPEED_MPH = 27
SPEED_MPS = SPEED_MPH / 2.237
LAG_FACTOR = 1.2
ADJUSTED_SPEED = SPEED_MPS * LAG_FACTOR


def generate_launch_description():
    sun_azimuth = 60.0
    sun_elevation = 15.0

    config_str = next(
        (
            config_param[5:]
            for config_param in sys.argv
            if config_param.startswith("key:=")
        ),
        "",
    )

    if len(config_str) == 1:
        config_param = scenario_mapper(ord(config_str.upper()) - ord("A"))
    else:
        config_param = training_mapper(config_str)

    location = side_map[config_param[0]]

    descriptions = [
        # Spawn Settings
        DeclareLaunchArgument(name="town", default_value="Town10HD_Opt"),
        DeclareLaunchArgument(name="spawn_point", default_value=location[0]),
        DeclareLaunchArgument(name="goal", default_value=location[1]),
        # Driving Settings
        DeclareLaunchArgument(name="target_speed", default_value=str(ADJUSTED_SPEED)),
        DeclareLaunchArgument(name="avoid_pedestrian", default_value="False"),
        # Recorder Settings
        DeclareLaunchArgument(name="record_gaze", default_value="False"),
        DeclareLaunchArgument(name="draw_gaze", default_value="False"),
        DeclareLaunchArgument(name="n", default_value="0"),
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
                "scenario_number": config_str,
                "participant_number": LaunchConfiguration("n"),
                "record_gaze": LaunchConfiguration("record_gaze"),
                "draw_gaze": LaunchConfiguration("draw_gaze"),
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
                {"spawn_location": config_param[0]},
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
        if name == "train":
            config["num"] = 0
            config["props"] = [{"blueprint": "warningaccident"}]

            if names[0][0] == "l":
                config["props"]["spawn"] = "center_right"
                config["props"]["spawn"] = "far_right"
                config["props"]["spawn"] = "center"
                config["props"]["spawn"] = "right"
            elif names[0][0] == "r":
                config["props"]["spawn"] = "near_left_margin"
                config["props"]["spawn"] = "center_right"
                config["props"]["spawn"] = "left"
                config["props"]["spawn"] = "center_right"
            continue
        config["pedestrians"].append(
            build_pedestrian(names[0], i, name, SPEED_MPS)
        )

    return config


side_map = {
    "ls": ("88.0, -16.6, 1.0, 0, 0, 180", "-5.5, -16.6, 0.0, 0, 0, 180"),
    "rs": ("88.0, -13.4, 1.0, 0, 0, 180", "-5.5, -13.2, 0.0, 0, 0, 180"),
    "lt": ("105.2, 30.0, 1.0, 0, 0, 105", "30.4, 64.4, 1.0, 0, 0, 180"),
    "rt": ("109.7, 30.0, 1.0, 0, 0, 105", "30.4, 67.8, 1.0, 0, 0, 180"),
}


def scenario_mapper(key):
    location = "rs" if key < 3 else "ls" if key < 6 else "rt" if key < 9 else "lt"

    ped_labels = get_pedestrian_labels(key % 3)
    return [location, *ped_labels]


def training_mapper(key: str):
    location = (
        "rs" if key == "t0" else "ls" if key == "t1" else "rt" if key == "t2" else "lt"
    )

    return [location, "train"]


if __name__ == "__main__":
    generate_launch_description()
