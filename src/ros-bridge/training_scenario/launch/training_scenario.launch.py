from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        DeclareLaunchArgument(name="town", default_value="Town01"), # TODO: Change this to HD_opt10 or whatever
        DeclareLaunchArgument(name="role_name", default_value="ego_vehicle" ),
        DeclareLaunchArgument(name="rain", default_value="False"),
        DeclareLaunchArgument(name="intensity", default_value="0.70"),
        DeclareLaunchArgument(name="sun_azimuth", default_value="0.50"),
        DeclareLaunchArgument(name="sun_elevation", default_value="0.05"),
        DeclareLaunchArgument(name="pedestrian_number", default_value="10"),
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
