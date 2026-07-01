import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_car')
    
    # Path to URDF/XACRO
    xacro_file = os.path.join(pkg_share, 'urdf', 'car.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Gazebo launch file path
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')

    # Path to custom world
    world_path = os.path.join(pkg_share, 'worlds', 'obstacle_world.world')

    # Gazebo Launch Description
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': world_path}.items()
    )

    # Spawn Entity Node
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'autonomous_car', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # Depthimage to Laserscan Node
    # Converts the Gazebo simulated camera/depth/image_raw to /scan
    node_depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('image', '/camera/depth/image_raw'),
            ('camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan')
        ],
        parameters=[{
            'output_frame': 'camera_link',
            'range_min': 0.1,
            'range_max': 8.0,
            'scan_height': 5, # scan rows in the image
        }],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        launch_gazebo,
        node_spawn_entity,
        node_depthimage_to_laserscan
    ])
