import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Get the package share directory
    pkg_path = get_package_share_directory('final_robot_ros')
    
    # 2. Path to your URDF file in the install folder
    urdf_file = os.path.join(pkg_path, 'urdf', 'final_robot_ros.urdf')

    # 3. Read the URDF content safely
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([

        # Robot State Publisher: Broadcasts the TF tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),

        # Joint State Publisher GUI: Allows manual control of joints in RViz
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz2: The visualization tool
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])