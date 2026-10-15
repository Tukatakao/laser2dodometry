import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():

    pkg_path = get_package_share_directory('laserslamodometry')
    urdf_path = os.path.join(pkg_path, 'urdf', 'xiaoche.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

   

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
       
    ])