import json
import os
import random
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

    config_str = next(
        (
            config_param[5]
            for config_param in sys.argv
            if config_param.startswith("key:=")
        ),
        "",
    )

    print(config_str)

    config_param = scenario_mapper(ord(config_str.upper()) - ord("A"))

    print(config_str)
    print(config_param)

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
        config["pedestrians"].append(build_pedestrian(names[0], i, name))

    return config


def build_pedestrian(location, n, kind):
    LABELS = {
        "soldier": 1,
        "police officer": 30,
        "orphan": 12,
        "terrorist": 15,
        "rapist": 16,
        "pedophile": 26,
        "judge": 20,
        "billionaire": 23,
        "celebrity": 25,
    }
    RT = 2.5

    pedestrian = {
        "actions": [
            {"yaw": 95.0 if n == 0 else -90.0},
            {"speed": 0.0, "tdelay": RT - 0.2},
        ],
        "yaw": 90.0 if n == 0 else -90.0,
    }

    if location[0] == "r":
        pedestrian["spawn"] = "far_left_margin"
    else:
        pedestrian["spawn"] = "near_left_margin"

    if location == "rs":
        pedestrian["actions"][0]["speed"] = 2.4 if n == 0 else 2.2
    if location == "rt":
        pedestrian["actions"][0]["speed"] = 2.4 if n == 0 else 1.4
    elif location == "ls":
        pedestrian["actions"][0]["speed"] = 1.2 if n == 0 else 1.4
    elif location == "lt":
        pedestrian["actions"][0]["speed"] = 1.0

    if n == 0:
        pedestrian["spawn"] = "right"

    pedestrian["actions"].append(
        {"speed": 2.0, "yaw": -pedestrian["actions"][0]["yaw"], "tdelay": 0.1}
    )

    pedestrian["actions"][0]["mdelay"] = RT * ADJUSTED_SPEED
    pedestrian["blueprint"] = LABELS[kind]

    return pedestrian


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


def get_pedestrian_labels(delta):
    value_arr = []
    if delta == 2:
        rand = random.choice([0, 2])
        value_arr = [rand, 2 - rand]
    elif delta == 0:
        rand = random.choice(range(2))
        value_arr = [rand] * 2
    else:
        rand = random.choice(range(2))
        if rand == 2:
            value_arr = [2, 1]
        elif rand == 0:
            value_arr = [0, 1]
        else:
            rand = random.choice([0, 2])
            value_arr = [1, rand]

    return [get_pedestrian(value) for value in value_arr]


def get_pedestrian(value):
    if value == 2:
        return get_high_value_pedestrian()
    elif value == 1:
        return get_mid_value_pedestrian()
    else:
        return get_low_value_pedestrian()


def get_high_value_pedestrian():
    return random.choice(["soldier", "police officer", "orphan"])


def get_low_value_pedestrian():
    return random.choice(["terrorist", "rapist", "pedophile"])


def get_mid_value_pedestrian():
    return random.choice(["judge", "billionaire", "celebrity"])


if __name__ == "__main__":
    generate_launch_description()
