import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from apriltag_msgs.msg import AprilTagDetectionArray
import time
import math

class Locomotion(Node):

    def __init__(self):
        super().__init__('turtlebot3_locomotion')

        # --- PUBLISHERS ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # --- SUBSCRIBERS ---
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10
        )

        # --- STATE VARIABLES ---
        self.min_front_dist = 0.4  # Distance to stop from wall
        self.seen_tags = set()     # Memory of tags we have already scanned
        
        # Flags for State Machine
        self.is_blocked = False
        self.is_scanning_tag = False
        self.scan_start_time = 0.0

    def create_twist_msg(self, x, z):
        t = Twist()
        t.linear.x = float(x)
        t.angular.z = float(z)
        return t

    def scan_callback(self, msg):
        """
        Handles Lidar data. 
        Silent logic.
        """
        front_cone = msg.ranges[0:20] + msg.ranges[-20:]
        
        valid_ranges = [
            r for r in front_cone 
            if not math.isnan(r) and not math.isinf(r) and r > 0.01
        ]
        
        if valid_ranges:
            min_distance = min(valid_ranges)
            if min_distance < self.min_front_dist:
                self.is_blocked = True
            else:
                self.is_blocked = False
        else:
            self.is_blocked = False
        
        self.decide_movement()

    def tag_callback(self, msg):
        """
        Handles AprilTag detections.
        Only prints if the tag is NEW.
        """
        if len(msg.detections) > 0:
            tag_id = msg.detections[0].id
            
            # Check if we have seen this tag before
            if tag_id not in self.seen_tags:
                # If we are currently doing a spin for a previous tag, 
                # we might still "see" a new one. 
                # This logic ensures we capture it and print it.
                
                self.seen_tags.add(tag_id)
                
                # THIS IS THE ONLY PRINT STATEMENT IN THE CODE
                print(f"Tag detected {tag_id}")

                # Trigger the scan/spin behavior
                if not self.is_scanning_tag:
                    self.is_scanning_tag = True
                    self.scan_start_time = time.time()

    def decide_movement(self):
        """
        Main State Machine (Silent)
        """
        twist = Twist()

        # PRIORITY 1: Scanning Routine (7 Seconds)
        if self.is_scanning_tag:
            elapsed_time = time.time() - self.scan_start_time
            
            if elapsed_time < 7.0:
                twist = self.create_twist_msg(0.0, 0.5) # Spin
            else:
                self.is_scanning_tag = False
                twist = self.create_twist_msg(0.0, 0.0)
        
        # PRIORITY 2: Obstacle Avoidance
        elif self.is_blocked:
            twist = self.create_twist_msg(0.0, 0.5) # Turn

        # PRIORITY 3: Move Forward
        else:
            twist = self.create_twist_msg(0.20, 0.0) # Forward

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    node = Locomotion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
