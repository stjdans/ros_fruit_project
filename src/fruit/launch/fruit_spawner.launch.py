from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fruit_robot_project',
            executable='fruit_spawner',
            name='fruit_spawner',
            output='screen'
        ),
    ])
