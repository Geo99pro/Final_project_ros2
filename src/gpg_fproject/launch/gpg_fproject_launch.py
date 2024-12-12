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
    
    rviz_config_file = PathJoinSubstitution([FindPackageShare("gpg_remote"), "gpg_remote.rviz"])
    print(f'RVIZ config file is located at: {rviz_config_file}')
    
    time_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments="/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        output="both"
    )

    controller_node = Node(
        package='gpg_fproject_controller',
        executable='controller',
        output='screen',
    )

    reach_goal_node = Node(
        package='gpg_fproject_controller',
        executable='reach_goal',
        output='screen',
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
    
    rviz_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='log',
                    arguments=['-d', rviz_config_file],)

    sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments = {'gz_args': f"{gazebo_model_path} -r",
                            'on_exit_shutdown': 'True'}.items(),
    )

    robot_controller_spawner = Node(package="controller_manager",
                                    executable="spawner",
                                    arguments=["diff_drive_controller", "-c", "/controller_manager"],)
    
    joint_state_broadcaster_spawner = Node(package="controller_manager",
                                        executable="spawner",
                                        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],)

    servo_controller_spawner = Node(package="controller_manager",
                                    executable="spawner",
                                    arguments=["servo_controller", "--controller-manager", "/controller_manager"],)

    robot_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description'],
        output='screen',
    )

    image_bridge_node = Node(package='ros_gz_bridge',
                            executable='image_bridge',
                            name='image_bridge',
                            arguments=[ "/image@sensor_msgs/msg/Image@gz.msgs.Image",
                                        "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"],
                            output="screen",
    )   

    delay_controllers_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner, servo_controller_spawner],))

    nodes = [time_bridge, 
            image_node, 
            user_node,
            controller_node,
            reach_goal_node, 
            robot_publisher, 
            sim_node, 
            rviz_node, 
            robot_controller_spawner, 
            joint_state_broadcaster_spawner, 
            servo_controller_spawner, 
            image_bridge_node, 
            delay_controllers_after_joint_state_broadcaster_spawner]

    return LaunchDescription(nodes)