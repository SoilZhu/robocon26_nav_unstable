import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    gazebo_share_dir = get_package_share_directory('robocon_gazebo')
    tf_broadcaster_share_dir = get_package_share_directory('robocon_tf_broadcaster')
    waypoints_share_dir = get_package_share_directory('robocon_waypoints')
    nav_share_dir = get_package_share_directory('robocon_nav')
    
    # Load offsets configuration
    tf_config_file = os.path.join(tf_broadcaster_share_dir, 'config', 'tf_offsets.yaml')
    lidar_offset = [0.1, 0.0, 0.3, 0.0, 0.0, 0.0]
    sensor_offset = [-0.15, -0.05, -0.02, 0.0, 0.0, 0.0]
    try:
        with open(tf_config_file, 'r') as f:
            yaml_data = yaml.safe_load(f)
            params = yaml_data['/robocon_tf_broadcaster']['ros__parameters']
            lidar_offset = params.get('lidar_offset', lidar_offset)
            sensor_offset = params.get('sensor_offset', sensor_offset)
    except Exception as e:
        print(f"Failed to parse offsets from yaml in sim.launch.py: {e}")
        
    # Load navigation waypoints configuration
    waypoints_file = os.path.join(waypoints_share_dir, 'config', 'waypoints.yaml')
    ref_x = 2.0
    color_trigger = 1
    try:
        with open(waypoints_file, 'r') as f:
            yaml_data = yaml.safe_load(f)
            params = yaml_data['/robocon_nav_node']['ros__parameters']
            ref_x = params.get('ref_x_coordinate', ref_x)
            color_trigger = params.get('color_trigger_val', color_trigger)
    except Exception as e:
        print(f"Failed to parse parameters from yaml in sim.launch.py: {e}")

    # Launch configuration arguments
    run_nav_arg = DeclareLaunchArgument(
        'run_nav',
        default_value='true',
        description='Whether to start the navigation system alongside the simulation'
    )
    
    # Xacro parser
    xacro_file = os.path.join(gazebo_share_dir, 'urdf', 'robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )
    
    # Gazebo simulation launch
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir, 'launch', 'gazebo.launch.py')
        )
    )
    
    # Spawn Entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robocon_robot'],
        output='screen'
    )
    
    # Sim Bridge and Mock node
    sim_bridge = Node(
        package='robocon_gazebo',
        executable='sim_bridge',
        name='sim_bridge_node',
        output='screen',
        parameters=[
            {'lidar_offset': lidar_offset},
            {'sensor_offset': sensor_offset},
            {'ref_x_coordinate': ref_x},
            {'color_trigger_val': color_trigger}
        ]
    )
    
    # Optional launch of navigation system (which includes static tf_broadcaster)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share_dir, 'launch', 'nav.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('run_nav'))
    )
    
    return LaunchDescription([
        run_nav_arg,
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        sim_bridge,
        nav_launch
    ])
