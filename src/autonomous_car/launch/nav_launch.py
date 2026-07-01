import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_car')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # Paths
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav_launch_script = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    # Include Nav2 Navigation
    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_script),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # Static TF Publisher (map -> odom)
    # This acts as a dummy localization node so that RViz and Nav2 can match frames
    node_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    return LaunchDescription([
        node_static_tf,
        launch_navigation
    ])
