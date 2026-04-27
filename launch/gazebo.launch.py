import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('final_robot_ros')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'final_robot_ros.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Bulletproof mesh loading: replace package:// with file:// absolute paths
    robot_desc = robot_desc.replace('package://final_robot_ros', 'file://' + pkg_share)

    # Expose package meshes AND worlds to Ignition resource search
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = (
        os.path.join(pkg_share, '..') + ':' +
        os.path.join(pkg_share, 'worlds')
    )

    world_file = os.path.join(pkg_share, 'worlds', 'hospital.sdf')
    slam_params_file = os.path.join(
        pkg_share,
        'config',
        'mapper_params_online_async.yaml'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r --render-engine ogre ' + world_file}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-string', robot_desc,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.02'
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Teleoperation: ROS /cmd_vel -> Gazebo /cmd_vel.
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Odometry and TF from the DiffDrive plugin.
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            # 2D LiDAR.
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            # RGB camera. Gazebo publishes /camera and /camera_info; remap
            # them to the ROS image pipeline names expected by RViz/Nav tools.
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # Simulation clock for use_sim_time nodes.
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        remappings=[
            ('/camera', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),
        gz_sim,
        spawn_entity,
        robot_state_publisher,
        bridge,
        slam_toolbox
    ])
