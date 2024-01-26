from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='training_scenario',
            namespace='training_scenario1',
            executable='training_scenario',
            output='screen',
            name='train'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
