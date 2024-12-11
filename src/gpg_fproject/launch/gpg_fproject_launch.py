import os
import xacro

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    This function is used to generate the launch description for the project"""

    gazebo_model_path = os.path.join(get_package_share_path('gazebo_world'), 'world', 'fproject_world.sdf')
    print(f'Gazebo model path is located at: {gazebo_model_path}')

    urdf = get_package_share_directory('gpg_fproject') / 'robot.urdf'
    print(f'URDF file is located at: {urdf}')

    robot_description_config = xacro.process_file(urdf).toxml()
    robot_description = {'robot_description': robot_description_config}

    time_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="both"
    )

    image_node = Node(
        package='gpg_fproject',
        executable='image_node',
        output='screen'
    )

    user_node = Node(
        package='gpg_fproject',
        executable='user_node',
        output='screen'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )
    
