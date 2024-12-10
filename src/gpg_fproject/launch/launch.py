import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file_name = 'robot.urdf'
    print(f'urdf file name in use is: {urdf_file_name}')
    urdf = os.path.join(get_package_share_directory('gpg_fproject'), urdf_file_name)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]
        )
    ])