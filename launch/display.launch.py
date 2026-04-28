import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Get the package share directory
    pkg_path = get_package_share_directory('final_robot_ros')
    
    # 2. Path to your URDF file
    urdf_file = os.path.join(pkg_path, 'urdf', 'final_robot_ros.urdf')

    # 3. Read the URDF content
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    # 4. Define the LaunchConfiguration reference (The missing piece!)
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Declare the argument so it can be changed via command line
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': use_sim_time,
            }]
        ),

        # Joint State Publisher GUI
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])