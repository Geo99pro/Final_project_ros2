from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gpg_fproject_controller',
            executable='reach_goal',
            name='reach_goal',
            output='screen'
        ),
        Node(
            package='gpg_fproject_controller',
            executable='controller',
            name='controller',
            output='screen'
        )
    ])
