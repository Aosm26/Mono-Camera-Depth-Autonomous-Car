import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_car')
    
    # Set Gazebo environment variables to find custom models and resources
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    install_share_path = os.path.dirname(pkg_share)
    
    if install_share_path not in gazebo_model_path:
        os.environ['GAZEBO_MODEL_PATH'] = install_share_path + (':' + gazebo_model_path if gazebo_model_path else '')
    if install_share_path not in gazebo_resource_path:
        os.environ['GAZEBO_RESOURCE_PATH'] = install_share_path + (':' + gazebo_resource_path if gazebo_resource_path else '')
    
    # Declare world_name argument
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='road_world.world',
        description='Name of the world file to load'
    )
    
    # Declare use_sim_depth argument
    use_sim_depth_arg = DeclareLaunchArgument(
        'use_sim_depth',
        default_value='false',
        description='Whether to launch simulated depth to laserscan node'
    )
    
    # Path to URDF/XACRO
    xacro_file = os.path.join(pkg_share, 'urdf', 'car.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # Robot State Publisher Node
    node_robot_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Gazebo launch file path
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')

    # Setup OpaqueFunction to dynamically resolve world file path as a plain Python string
    # (Fixes shell-splitting/quoting issues with spaces in the workspace path)
    def launch_setup(context, *args, **kwargs):
        world_name = context.perform_substitution(LaunchConfiguration('world_name'))
        # Wrap the path in double quotes so that the shell-executed gzserver command handles spaces correctly
        world_path = f'"{os.path.join(pkg_share, "worlds", world_name)}"'
        
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={'world': world_path}.items()
            )
        ]

    # Save the processed URDF to a file to avoid transient topic subscription issues in spawn_entity
    urdf_file = os.path.join(pkg_share, 'urdf', 'car.urdf')
    with open(urdf_file, 'w') as f:
        f.write(robot_desc)

    # Spawn Entity Node
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_file, '-entity', 'autonomous_car', '-x', '-12.0', '-y', '-10.0', '-z', '0.15', '-Y', '0.0'],
        output='screen'
    )

    # Depthimage to Laserscan Node
    # Converts the Gazebo simulated camera/depth/image_raw to /scan
    node_depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        condition=IfCondition(LaunchConfiguration('use_sim_depth')),
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
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
        world_name_arg,
        use_sim_depth_arg,
        node_robot_robot_state_publisher,
        OpaqueFunction(function=launch_setup),
        node_spawn_entity,
        node_depthimage_to_laserscan
    ])

