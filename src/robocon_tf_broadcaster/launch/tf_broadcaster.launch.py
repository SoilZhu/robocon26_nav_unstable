import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory('robocon_tf_broadcaster')
    config_file = os.path.join(package_share_dir, 'config', 'tf_offsets.yaml')
    
    # Load offsets from yaml file for TF configuration
    lidar_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    sensor_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    try:
        with open(config_file, 'r') as f:
            yaml_data = yaml.safe_load(f)
            params = yaml_data['/robocon_tf_broadcaster']['ros__parameters']
            lidar_offset = params.get('lidar_offset', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            sensor_offset = params.get('sensor_offset', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    except Exception as e:
        print(f"Failed to parse offsets from yaml in tf_broadcaster: {e}")
        
    # Since Point-LIO publishes camera_init -> aft_mapped (LiDAR link),
    # we need static transform for aft_mapped (LiDAR) -> base_link (chassis center).
    # This is the inverse of the lidar_offset (lidar pose relative to base_link).
    # For translations, we negate the offset values.
    lidar_tf_args = [
        str(-lidar_offset[0]), str(-lidar_offset[1]), str(-lidar_offset[2]),
        str(-lidar_offset[5]), str(-lidar_offset[4]), str(-lidar_offset[3]), # yaw, pitch, roll
        'aft_mapped', 'base_link'
    ]
    
    # Static transform for base_link -> color_sensor_link
    sensor_tf_args = [
        str(sensor_offset[0]), str(sensor_offset[1]), str(sensor_offset[2]),
        str(sensor_offset[5]), str(sensor_offset[4]), str(sensor_offset[3]), # yaw, pitch, roll
        'base_link', 'color_sensor_link'
    ]
    
    # TF static publishers
    lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf_publisher',
        arguments=lidar_tf_args
    )
    
    sensor_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensor_static_tf_publisher',
        arguments=sensor_tf_args
    )
    
    return LaunchDescription([
        lidar_tf_node,
        sensor_tf_node
    ])
