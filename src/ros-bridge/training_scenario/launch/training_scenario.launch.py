import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription([
        DeclareLaunchArgument(name="town", default_value="Town01"), # TODO: Change this to HD_opt10 or whatever
        DeclareLaunchArgument(name="role_name", default_value="ego_vehicle" ),
        DeclareLaunchArgument(name="rain", default_value="False"),
        DeclareLaunchArgument(name="intensity", default_value="0.70"),
        DeclareLaunchArgument(name="sun_azimuth", default_value="0.50"),
        DeclareLaunchArgument(name="sun_elevation", default_value="0.05"),
        DeclareLaunchArgument(name="pedestrian_number", default_value="10"),
        DeclareLaunchArgument(
            name='spawn_point',
            default_value='396.0,-313.0,2.0,0,0,90'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ad_demo'), 'carla_ad_demo.launch.py')
            ),
            launch_arguments={
                'role_name': LaunchConfiguration('role_name'),
                'spawn_point': LaunchConfiguration('spawn_point'),
                'avoid_risk': "True"
            }.items()
        ),

        Node(
            package='training_scenario',
            namespace='training_scenario1',
            executable='training_scenario',
            output='screen',
            name='train',
            parameters=[
                {
                    'role_name': LaunchConfiguration('role_name')
                }
            ]
        ),
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
