import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import os

class ValidationNode(Node):
    def __init__(self):
        super().__init__('safety_validation_node')
        
        # Declare explicit parameters to allow dynamic remapping per drone
        self.declare_parameter('student_topic', '/drone11/setpoint_position/local')
        self.declare_parameter('drone_topic', '/drones/edu11/setpoint_position/local')
        self.declare_parameter('boundaries_file', '')
        
        student_topic = self.get_parameter('student_topic').value
        drone_topic = self.get_parameter('drone_topic').value
        boundaries_file = self.get_parameter('boundaries_file').value
        
        # Load boundaries
        self.boundaries = {'x_min': -5.0, 'x_max': 5.0, 'y_min': -5.0, 'y_max': 5.0, 'z_min': 0.0, 'z_max': 3.0}
        if boundaries_file and os.path.exists(boundaries_file):
            with open(boundaries_file, 'r') as f:
                self.boundaries.update(yaml.safe_load(f))
            self.get_logger().info(f"Loaded boundaries from {boundaries_file}")
        else:
            self.get_logger().warn("No valid boundaries file provided, using defaults.")
            
        # Subs and Pubs
        self.sub = self.create_subscription(PoseStamped, student_topic, self.pose_callback, 10)
        self.pub = self.create_publisher(PoseStamped, drone_topic, 10)
        
        self.get_logger().info(f"Validation Node checking {student_topic} -> {drone_topic}")

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # Validate against boundaries
        if (self.boundaries['x_min'] <= x <= self.boundaries['x_max'] and
            self.boundaries['y_min'] <= y <= self.boundaries['y_max'] and
            self.boundaries['z_min'] <= z <= self.boundaries['z_max']):
            
            # Safe to publish to the bridge
            self.pub.publish(msg)
        else:
            self.get_logger().warn(f"SAFETY VIOLATION DETECTED: Command out of bounds! Dropping command. ({x}, {y}, {z})")

def main(args=None):
    rclpy.init(args=args)
    node = ValidationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
