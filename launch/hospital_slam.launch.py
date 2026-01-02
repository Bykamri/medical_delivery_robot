from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('medical_delivery_robot')

    xacro_file = os.path.join(share_dir, 'urdf', 'medical_delivery.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    world_file = os.path.join(share_dir, 'worlds', 'hospital_static_human.world')
    slam_params_file = os.path.join(share_dir, 'config', 'slam_params.yaml')
    rviz_config_file = os.path.join(share_dir, 'config', 'slam.rviz')

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': True}
        ]
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'source_list': ['joint_states']}
        ]
    )

    # Gazebo Server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'pause': 'false'
        }.items()
    )

    # Gazebo Client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Spawn Robot Entity
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'medical_delivery',
            '-topic', 'robot_description',
            '-x', '1.0',
            '-y', '16.0',
            '-z', '0.1',
            '-Y', '-1.575'
        ],
        output='screen'
    )

    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': True}
        ]
    )

    # RViz for SLAM Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Robot Move Node (uncomment if needed)
    robot_move_node = Node(
        package='medical_delivery_robot',
        executable='robot_move',
        name='robot_move_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Robot Move Enhanced Node (improved obstacle avoidance with rotation)
    robot_move_enhanced_node = Node(
        package='medical_delivery_robot',
        executable='robot_move_enhanced',
        name='robot_move_enhanced_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        slam_toolbox_node,
        rviz_node,
        # robot_move_enhanced_node  # Using enhanced version with better rotation logic
    ])
