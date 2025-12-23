from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='medical_delivery_robot',
            executable='robot_move',
            name='robot_move_node',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),
    ])
