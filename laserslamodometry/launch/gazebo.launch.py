import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():

    pkg_path = get_package_share_directory('laserslamodometry')
    urdf_path = os.path.join(pkg_path, 'urdf', 'xiaoche.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so','-s','libgazebo_ros_init.so'],
            output='screen' 
        ),  
        TimerAction(period=5.0, actions=
            [

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description},
                        {'use_sim_time': True}]
            ),
            ]
        ),
        TimerAction(period=10.0, actions=[   
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'xwdc', '-file', urdf_path],
            output='screen'
        ),]),
    ])