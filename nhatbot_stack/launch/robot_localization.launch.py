import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    map_path = os.path.join(get_package_share_directory('nhatbot_stack'), 'maps', 'map.yaml')
    map_file_arg = DeclareLaunchArgument(
        'map_file', 
        default_value=map_path, 
        description='Path to map.yaml')

    amcl_config_path = os.path.join(get_package_share_directory('nhatbot_stack'), 'config', 'amcl_config.yaml')
    amcl_config_arg = DeclareLaunchArgument(
        'amcl_config', 
        default_value=amcl_config_path, 
        description='Path to AMCL config yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': LaunchConfiguration('map_file')
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('amcl_config')]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    return LaunchDescription([
        map_file_arg,
        amcl_config_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node
    ])





