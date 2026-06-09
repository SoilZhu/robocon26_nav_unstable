import rclpy
from rclpy.node import Node
import yaml
import math
import os

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, UInt8
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# State machine constants
STATE_IDLE = 0
STATE_TRAVERSING = 1
STATE_ROTATING = 2
STATE_HEIGHT_ADJUST = 3
STATE_WAITING = 4
STATE_FINISHED = 5

class RoboconNavNode(Node):
    def __init__(self):
        super().__init__('robocon_nav_node')
        
        # Declare parameters
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('auto_start', True)
        
        # Controller tuning parameters
        self.declare_parameter('kp_linear', 1.5)
        self.declare_parameter('kp_yaw', 2.0)
        self.declare_parameter('max_linear_speed', 1.5)      # m/s
        self.declare_parameter('min_linear_speed', 0.1)      # m/s
        self.declare_parameter('max_decstd_accel', 1.0)      # m/s^2 (max deceleration)
        self.declare_parameter('max_yaw_speed', 1.5)         # rad/s
        self.declare_parameter('min_yaw_speed', 0.15)        # rad/s
        
        self.declare_parameter('arrive_dist_threshold', 0.05) # 5 cm
        self.declare_parameter('align_yaw_threshold', 0.03)   # ~1.7 degrees
        self.declare_parameter('height_timeout', 8.0)        # seconds
        
        # Color sensor calibration parameters
        self.declare_parameter('color_trigger_val', 1)        # value that triggers calibration
        self.declare_parameter('ref_x_coordinate', 2.0)       # absolute X position of reference line (A)
        
        # Get parameters
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
        
        self.kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        self.kp_yaw = self.get_parameter('kp_yaw').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.min_linear_speed = self.get_parameter('min_linear_speed').get_parameter_value().double_value
        self.max_decstd_accel = self.get_parameter('max_decstd_accel').get_parameter_value().double_value
        self.max_yaw_speed = self.get_parameter('max_yaw_speed').get_parameter_value().double_value
        self.min_yaw_speed = self.get_parameter('min_yaw_speed').get_parameter_value().double_value
        
        self.arrive_dist_threshold = self.get_parameter('arrive_dist_threshold').get_parameter_value().double_value
        self.align_yaw_threshold = self.get_parameter('align_yaw_threshold').get_parameter_value().double_value
        self.height_timeout = self.get_parameter('height_timeout').get_parameter_value().double_value
        
        self.color_trigger_val = self.get_parameter('color_trigger_val').get_parameter_value().integer_value
        self.ref_x_coordinate = self.get_parameter('ref_x_coordinate').get_parameter_value().double_value
        
        # Internal states
        self.current_state = STATE_IDLE
        self.waypoints = []
        self.current_waypoint_idx = 0
        
        # Robot pose (from TF)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False
        
        # Offsets for absolute coordinate calibration
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.calibration_triggered = False
        
        # Height state
        self.height_reached = False
        self.last_target_height = 0.0
        
        # State machine timers/tracking
        self.state_start_time = None
        self.traverse_target_yaw = 0.0
        
        # Load waypoints
        self.load_waypoints()
        
        # TF Buffer & Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriptions
        self.height_reached_sub = self.create_subscription(
            Bool,
            '/height_reached',
            self.height_reached_callback,
            10
        )
        
        self.color_sensor_sub = self.create_subscription(
            UInt8,
            '/color_sensor_state',
            self.color_sensor_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_height_pub = self.create_publisher(Float32, '/target_height', 10)
        
        # Control loop timer (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Robocon Waypoint Navigation Node (with TF & Calibration) Initialized.')

    def load_waypoints(self):
        if not self.waypoints_file or not os.path.exists(self.waypoints_file):
            self.get_logger().warn(f"Waypoints file '{self.waypoints_file}' not found or not specified. Creating dummy waypoints.")
            # Default fallback waypoints
            self.waypoints = [
                {'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'target_height': 0.0, 'wait_time': 1.0},
                {'x': 1.0, 'y': 1.0, 'yaw': 1.57, 'target_height': 0.3, 'wait_time': 2.0},
                {'x': 0.0, 'y': 1.0, 'yaw': 3.14, 'target_height': 0.0, 'wait_time': 1.0},
                {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'target_height': 0.0, 'wait_time': 1.0}
            ]
            return
            
        try:
            with open(self.waypoints_file, 'r') as f:
                data = yaml.safe_load(f)
                if data and '/robocon_nav_node' in data and 'ros__parameters' in data['/robocon_nav_node'] and 'waypoints' in data['/robocon_nav_node']['ros__parameters']:
                    self.waypoints = data['/robocon_nav_node']['ros__parameters']['waypoints']
                    self.get_logger().info(f"Successfully loaded {len(self.waypoints)} waypoints from {self.waypoints_file}")
                elif data and 'waypoints' in data:
                    self.waypoints = data['waypoints']
                    self.get_logger().info(f"Successfully loaded {len(self.waypoints)} waypoints from {self.waypoints_file}")
                else:
                    self.get_logger().error("No 'waypoints' key found in waypoints YAML file!")
        except Exception as e:
            self.get_logger().error(f"Failed to parse waypoints file: {e}")

    def height_reached_callback(self, msg: Bool):
        self.height_reached = msg.data

    def color_sensor_callback(self, msg: UInt8):
        if msg.data == self.color_trigger_val:
            if not self.calibration_triggered:
                # Trigger calibration using color sensor link transform
                try:
                    # Look up current sensor coordinate in camera_init frame
                    trans = self.tf_buffer.lookup_transform(
                        'camera_init',
                        'color_sensor_link',
                        rclpy.time.Time()
                    )
                    sensor_x_raw = trans.transform.translation.x
                    
                    # Calculate X offset: X_map = X_raw + X_offset  =>  X_offset = X_ref - X_raw
                    self.x_offset = self.ref_x_coordinate - sensor_x_raw
                    self.calibration_triggered = True
                    self.get_logger().info(
                        f"CALIBRATION TRIGGERED! Sensor raw X in camera_init: {sensor_x_raw:.3f}m. "
                        f"Ref X: {self.ref_x_coordinate:.3f}m. New X Offset: {self.x_offset:.3f}m"
                    )
                except TransformException as ex:
                    self.get_logger().warn(f"Failed to lookup color_sensor_link to calculate calibration offset: {ex}")
        else:
            self.calibration_triggered = False

    def get_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def update_robot_pose(self):
        try:
            # Query base_link relative to camera_init (world frame)
            trans = self.tf_buffer.lookup_transform(
                'camera_init',
                'base_link',
                rclpy.time.Time()
            )
            self.robot_x = trans.transform.translation.x
            self.robot_y = trans.transform.translation.y
            
            # Extract yaw
            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            self.odom_received = True
            return True
        except TransformException as ex:
            # Throttling warnings
            self.get_logger().warn(f"TF Lookup camera_init -> base_link failed: {ex}", throttle_duration_sec=2.0)
            return False

    def control_loop(self):
        # Update robot pose via TF tree lookup
        if not self.update_robot_pose():
            return

        cmd = Twist()
        
        # Calculate corrected coordinates in map frame
        robot_x_map = self.robot_x + self.x_offset
        robot_y_map = self.robot_y + self.y_offset

        # If in idle and auto_start is true, transition to traversing first waypoint
        if self.current_state == STATE_IDLE:
            if self.auto_start and len(self.waypoints) > 0:
                self.get_logger().info("Starting navigation...")
                self.current_state = STATE_TRAVERSING
                self.current_waypoint_idx = 0
                self.traverse_target_yaw = self.robot_yaw
            else:
                self.publish_stop()
                return

        # Check if index is valid
        if self.current_waypoint_idx >= len(self.waypoints):
            self.current_state = STATE_FINISHED

        # Main State Machine
        if self.current_state == STATE_TRAVERSING:
            wp = self.waypoints[self.current_waypoint_idx]
            dx = wp['x'] - robot_x_map
            dy = wp['y'] - robot_y_map
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist > self.arrive_dist_threshold:
                # Proportional velocity
                v = self.kp_linear * dist
                
                # Deceleration limit: v <= sqrt(2 * a * d)
                v_decel = math.sqrt(2.0 * self.max_decstd_accel * dist)
                v = min(v, v_decel)
                
                # Apply limits
                v = min(v, self.max_linear_speed)
                v = max(v, self.min_linear_speed)
                
                # Direction of travel in map/world frame
                angle = math.atan2(dy, dx)
                vx_world = v * math.cos(angle)
                vy_world = v * math.sin(angle)
                
                # Convert to robot local frame
                vx_robot = vx_world * math.cos(self.robot_yaw) + vy_world * math.sin(self.robot_yaw)
                vy_robot = -vx_world * math.sin(self.robot_yaw) + vy_world * math.cos(self.robot_yaw)
                
                # Heading controller (maintain traverse_target_yaw to prevent spinning)
                yaw_err = self.normalize_angle(self.traverse_target_yaw - self.robot_yaw)
                wz = self.kp_yaw * yaw_err
                # Apply angular speed limits
                wz = min(wz, self.max_yaw_speed)
                wz = max(wz, -self.max_yaw_speed)
                
                # Fill velocity command
                cmd.linear.x = vx_robot
                cmd.linear.y = vy_robot
                cmd.angular.z = wz
                
                self.cmd_vel_pub.publish(cmd)
                
                # Continuously publish last target height
                h_msg = Float32()
                h_msg.data = float(self.last_target_height)
                self.target_height_pub.publish(h_msg)
            else:
                self.get_logger().info(f"Arrived at position of waypoint {self.current_waypoint_idx + 1}")
                self.publish_stop()
                self.current_state = STATE_ROTATING
                
        elif self.current_state == STATE_ROTATING:
            wp = self.waypoints[self.current_waypoint_idx]
            yaw_err = self.normalize_angle(wp['yaw'] - self.robot_yaw)
            
            if abs(yaw_err) > self.align_yaw_threshold:
                wz = self.kp_yaw * yaw_err
                # Apply yaw limits and min speed to overcome friction
                if wz > 0:
                    wz = max(wz, self.min_yaw_speed)
                    wz = min(wz, self.max_yaw_speed)
                else:
                    wz = min(wz, -self.min_yaw_speed)
                    wz = max(wz, -self.max_yaw_speed)
                
                cmd.angular.z = wz
                self.cmd_vel_pub.publish(cmd)
                
                # Keep target height at last target height
                h_msg = Float32()
                h_msg.data = float(self.last_target_height)
                self.target_height_pub.publish(h_msg)
            else:
                self.get_logger().info(f"Aligned heading for waypoint {self.current_waypoint_idx + 1}")
                self.publish_stop()
                self.current_state = STATE_HEIGHT_ADJUST
                self.state_start_time = self.get_time_sec()
                self.height_reached = False # Reset reached flag to wait for MCU feedback
                
        elif self.current_state == STATE_HEIGHT_ADJUST:
            wp = self.waypoints[self.current_waypoint_idx]
            target_h = wp['target_height']
            self.last_target_height = target_h
            
            # Send height command
            h_msg = Float32()
            h_msg.data = float(target_h)
            self.target_height_pub.publish(h_msg)
            self.publish_stop() # Keep robot steady
            
            elapsed = self.get_time_sec() - self.state_start_time
            # Transition if height reached, or if timeout occurred
            if self.height_reached or elapsed >= self.height_timeout:
                if elapsed >= self.height_timeout:
                    self.get_logger().warn(f"Height adjustment timed out ({self.height_timeout}s) for waypoint {self.current_waypoint_idx + 1}")
                else:
                    self.get_logger().info(f"Height adjustment completed for waypoint {self.current_waypoint_idx + 1}")
                self.current_state = STATE_WAITING
                self.state_start_time = self.get_time_sec()
                
        elif self.current_state == STATE_WAITING:
            wp = self.waypoints[self.current_waypoint_idx]
            self.publish_stop()
            
            # Keep target height command active
            h_msg = Float32()
            h_msg.data = float(self.last_target_height)
            self.target_height_pub.publish(h_msg)
            
            elapsed = self.get_time_sec() - self.state_start_time
            if elapsed >= wp['wait_time']:
                self.get_logger().info(f"Waypoint {self.current_waypoint_idx + 1} finished.")
                self.current_waypoint_idx += 1
                if self.current_waypoint_idx < len(self.waypoints):
                    self.current_state = STATE_TRAVERSING
                    self.traverse_target_yaw = wp['yaw'] # Next traverse targets the orientation we just aligned to
                else:
                    self.current_state = STATE_FINISHED
                    
        elif self.current_state == STATE_FINISHED:
            self.publish_stop()
            # Publish height command continuously
            h_msg = Float32()
            h_msg.data = float(self.last_target_height)
            self.target_height_pub.publish(h_msg)
            self.get_logger().info("ALL WAYPOINTS COMPLETED. Navigation finished.", once=True)

    def publish_stop(self):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RoboconNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
