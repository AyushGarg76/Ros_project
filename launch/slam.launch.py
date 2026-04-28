import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('final_robot_ros')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    publish_robot_state = LaunchConfiguration('publish_robot_state')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    start_delay = LaunchConfiguration('start_delay')

    urdf_file = os.path.join(pkg_share, 'urdf', 'final_robot_ros.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    robot_desc = robot_desc.replace('package://final_robot_ros', 'file://' + pkg_share)

    default_params_file = os.path.join(
        pkg_share,
        'config',
        'mapper_params_online_async.yaml',
    )
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the slam_toolbox parameters file',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz for SLAM visualization',
        ),
        DeclareLaunchArgument(
            'publish_robot_state',
            default_value='false',
            description='Fallback only: Gazebo launch normally publishes robot_description and fixed TFs',
        ),
        DeclareLaunchArgument(
            'start_delay',
            default_value='2.0',
            description='Seconds to wait for Gazebo clock and TF before starting SLAM/RViz',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Full path to the RViz config file',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='slam_robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time,
            }],
            condition=IfCondition(publish_robot_state),
            output='screen',
        ),
        TimerAction(
            period=start_delay,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[
                        params_file,
                        {'use_sim_time': use_sim_time},
                    ],
                ),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': use_sim_time}],
                    condition=IfCondition(use_rviz),
                    output='screen',
                ),
            ],
        ),
    ])
