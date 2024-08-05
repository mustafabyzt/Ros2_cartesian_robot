import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('urdf_humble_test'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'model.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'verbose': 'true', 'factory': 'true'}.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', urdf_file, '-entity', 'robot'],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_entity,
        joint_state_publisher,
        robot_state_publisher,
        controller_manager,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[load_joint_state_broadcaster, load_joint_trajectory_controller],
            )
        )
    ])

