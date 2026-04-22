import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('final_robot_ros')
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'final_robot_ros.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Bulletproof mesh loading: replace package:// with file:// absolute paths
    robot_desc = robot_desc.replace('package://final_robot_ros', 'file://' + pkg_share)

    # Append package share to Ignition resource path so it can find meshes
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.path.join(pkg_share, '..')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r --render-engine ogre empty.sdf'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-string', robot_desc,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        spawn_entity,
        robot_state_publisher,
        bridge
    ])