import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params_file = "g29.yaml"
    params = os.path.join(
        get_package_share_directory("ros_g29_force_feedback"), "config", params_file
    )

    g29_ff = Node(
        package="ros_g29_force_feedback",
        executable="g29_force_feedback",
        name="g29_force_feedback",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[params, {"threshold": LaunchConfiguration("threshold")}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace", default_value="", description="Namespace for the node"
            ),
            DeclareLaunchArgument(name="threshold", default_value="0.0005"),
            g29_ff,
        ]
    )
