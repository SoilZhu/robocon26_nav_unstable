import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, UInt8
import tf2_ros
from tf2_geometry_msgs import TransformException

class SimBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_bridge_node')
        
        # Declare parameters for offsets (matching tf_offsets.yaml defaults)
        self.declare_parameter('lidar_offset', [0.1, 0.0, 0.3, 0.0, 0.0, 0.0])
        self.declare_parameter('sensor_offset', [-0.15, -0.05, -0.02, 0.0, 0.0, 0.0])
        self.declare_parameter('ref_x_coordinate', 2.0)
        self.declare_parameter('sensor_trigger_tolerance', 0.05) # 5 cm detection width
        self.declare_parameter('height_speed', 0.2) # m/s simulated elevator speed
        
        # Get parameters
        self.lidar_offset = self.get_parameter('lidar_offset').get_parameter_value().double_array_value
        self.sensor_offset = self.get_parameter('sensor_offset').get_parameter_value().double_array_value
        self.ref_x_coordinate = self.get_parameter('ref_x_coordinate').get_parameter_value().double_value
        self.sensor_trigger_tolerance = self.get_parameter('sensor_trigger_tolerance').get_parameter_value().double_value
        self.height_speed = self.get_parameter('height_speed').get_parameter_value().double_value
        
        # Internal states for height simulation
        self.current_height = 0.0
        self.target_height = 0.0
        self.last_height_update_time = self.get_clock().now()
        
        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.target_height_sub = self.create_subscription(
            Float32,
            '/target_height',
            self.target_height_callback,
            10
        )
        
        # Publishers
        self.height_reached_pub = self.create_publisher(Bool, '/height_reached', 10)
        self.color_sensor_pub = self.create_publisher(UInt8, '/color_sensor_state', 10)
        
        # Timer for publishing height state and color sensor state (50Hz)
        self.timer = self.create_timer(0.02, self.update_states_loop)
        
        self.get_logger().info("Decoupled Sim Bridge & Mock Node initialized.")

    def target_height_callback(self, msg: Float32):
        self.target_height = msg.data

    def odom_callback(self, msg: Odometry):
        # Extract robot 2D pose from odometry (camera_init -> base_link in simulation)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Compute LiDAR pose in camera_init frame
        # T_world_lidar = T_world_base * T_base_lidar
        lx_offset, ly_offset, lz_offset = self.lidar_offset[0], self.lidar_offset[1], self.lidar_offset[2]
        lyaw_offset = self.lidar_offset[5]
        
        lidar_x = x + lx_offset * math.cos(yaw) - ly_offset * math.sin(yaw)
        lidar_y = y + lx_offset * math.sin(yaw) + ly_offset * math.cos(yaw)
        lidar_z = msg.pose.pose.position.z + lz_offset
        lidar_yaw = yaw + lyaw_offset
        
        # Broadcast camera_init -> aft_mapped transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'camera_init'
        t.child_frame_id = 'aft_mapped'
        
        t.transform.translation.x = lidar_x
        t.transform.translation.y = lidar_y
        t.transform.translation.z = lidar_z
        
        # Convert Euler angles to quaternion
        half_yaw = lidar_yaw * 0.5
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half_yaw)
        t.transform.rotation.w = math.cos(half_yaw)
        
        self.tf_broadcaster.sendTransform(t)
        
        # Monitor color sensor position in world coordinate to simulate detection
        sx_offset, sy_offset = self.sensor_offset[0], self.sensor_offset[1]
        sensor_x = x + sx_offset * math.cos(yaw) - sy_offset * math.sin(yaw)
        
        # Check if color sensor crosses the absolute calibration line
        color_msg = UInt8()
        if abs(sensor_x - self.ref_x_coordinate) <= self.sensor_trigger_tolerance:
            color_msg.data = 1 # Trigger color sensor detection
        else:
            color_msg.data = 0
        self.color_sensor_pub.publish(color_msg)

    def update_states_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_height_update_time).nanoseconds / 1e9
        self.last_height_update_time = now
        
        # Safeguard dt
        if dt <= 0.0:
            dt = 0.02
            
        # Simulating elevator height mechanism movement
        diff = self.target_height - self.current_height
        step = self.height_speed * dt
        
        if abs(diff) > step:
            if diff > 0:
                self.current_height += step
            else:
                self.current_height -= step
            reached = False
        else:
            self.current_height = self.target_height
            reached = True
            
        # Publish reached status
        reached_msg = Bool()
        reached_msg.data = reached
        self.height_reached_pub.publish(reached_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
