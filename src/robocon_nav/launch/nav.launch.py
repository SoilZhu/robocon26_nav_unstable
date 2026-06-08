import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    waypoints_share_dir = get_package_share_directory('robocon_waypoints')
    waypoints_file = os.path.join(waypoints_share_dir, 'config', 'waypoints.yaml')
    
    # Load parameters from yaml file for configuration
    ref_x = 2.0
    color_trigger = 1
    
    try:
        with open(waypoints_file, 'r') as f:
            yaml_data = yaml.safe_load(f)
            params = yaml_data['/robocon_nav_node']['ros__parameters']
            ref_x = params.get('ref_x_coordinate', 2.0)
            color_trigger = params.get('color_trigger_val', 1)
    except Exception as e:
        print(f"Failed to parse parameters from yaml in launch file: {e}")
        
    # Include the separate tf_broadcaster launch script
    tf_broadcaster_share_dir = get_package_share_directory('robocon_tf_broadcaster')
    tf_launch_file = os.path.join(tf_broadcaster_share_dir, 'launch', 'tf_broadcaster.launch.py')
    
    tf_broadcaster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_launch_file)
    )
    
    # Navigation node
    nav_node = Node(
        package='robocon_nav',
        executable='nav_node',
        name='robocon_nav_node',
        output='screen',
        parameters=[
            {'waypoints_file': waypoints_file},
            {'auto_start': True},
            {'ref_x_coordinate': ref_x},
            {'color_trigger_val': color_trigger}
        ]
    )
    
    return LaunchDescription([
        tf_broadcaster_launch,
        nav_node
    ])
