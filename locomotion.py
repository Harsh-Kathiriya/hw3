import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.qos import qos_profile_sensor_data
import random

class LocomotionBrain(Node):

    def __init__(self):
        super().__init__('turtlebot3_brain')

        # --- State Machine Variable ---
        # The robot can be in one of three states:
        # "searching": Moving forward, looking for tags
        # "avoiding": Obstacle detected, turning
        # "scanning": Tag detected, doing a 360 spin
        self.robot_state = "searching"
        self.get_logger().info(f"Starting in state: {self.robot_state}")

        # --- Sensor Data Variables ---
        # We store the latest sensor data here
        self.current_obstacle_distance = 99.0
        self.is_tag_visible = False

        # --- Parameters ---
        # You can tune these values!
        self.obstacle_threshold = 0.5  # meters
        self.forward_speed = 0.1       # m/s
        self.turn_speed = 0.4          # rad/s
        self.scan_speed = 0.7          # rad/s (for 360 scan)
        
        # 2*pi radians is a full circle. 2*pi / 0.7 rad/s = ~8.97 seconds
        self.scan_duration_sec = 10.0  # (10s > 8.97s, so it's a full 360)
        self.scan_start_time = None

        # --- Publishers and Subscribers ---
        # Publisher for robot movement
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LaserScan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data) # Use sensor data QoS

        # Subscriber for AprilTag detections
        self.tag_subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10) # Default QoS is fine here

        # --- Main Decision Timer ---
        # We create a timer that runs the 'decision_loop' 10 times a second
        self.decision_timer = self.create_timer(0.1, self.decision_loop)

    def scan_callback(self, msg):
        """Callback for LaserScan data. Updates the obstacle distance."""
        # msg.ranges[0] is the beam directly in front.
        # Check for 'inf' (infinity) or 'nan' (not a number)
        if msg.ranges[0] == float('inf'):
            self.current_obstacle_distance = 99.0
        elif not msg.ranges[0] == float('nan'):
            self.current_obstacle_distance = msg.ranges[0]
        # if it's 'nan', we just keep the previous value

    def tag_callback(self, msg):
        """Callback for AprilTag data. Updates tag visibility."""
        if len(msg.detections) > 0:
            self.is_tag_visible = True
            # Log the tag ID *only* if we are not already scanning
            if self.robot_state != "scanning":
                self.get_logger().info(f"Tag {msg.detections[0].id} spotted!")
        else:
            self.is_tag_visible = False

    def decision_loop(self):
        """This is the "brain" of the robot, running 10x/sec."""
        
        # --- 1. State Transition Logic ---
        # First, decide if we need to CHANGE states
        
        is_blocked = self.current_obstacle_distance < self.obstacle_threshold

        if self.robot_state == "searching":
            if self.is_tag_visible:
                # Found a tag! Switch to scanning.
                self.robot_state = "scanning"
                self.get_logger().info("--> STATE: SCANNING")
                # Record the time we started scanning
                self.scan_start_time = self.get_clock().now()
            elif is_blocked:
                # Hit a wall! Switch to avoiding.
                self.robot_state = "avoiding"
                self.get_logger().info("--> STATE: AVOIDING")

        elif self.robot_state == "avoiding":
            if not is_blocked:
                # Path is clear. Go back to searching.
                self.robot_state = "searching"
                self.get_logger().info("--> STATE: SEARCHING (path is clear)")

        elif self.robot_state == "scanning":
            # Check if our scan time is up
            if self.scan_start_time is None:
                 self.robot_state = "searching" # Safety check
                 return
                 
            elapsed_time = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
            if elapsed_time > self.scan_duration_sec:
                # Scan is complete. Go back to searching.
                self.robot_state = "searching"
                self.get_logger().info(f"--> STATE: SEARCHING (scan complete)")
                self.scan_start_time = None

        # --- 2. Action Logic ---
        # Now, set motor commands based on our CURRENT state
        
        twist_msg = Twist()

        if self.robot_state == "searching":
            # Move forward
            twist_msg.linear.x = self.forward_speed
            twist_msg.angular.z = 0.0

        elif self.robot_state == "avoiding":
            # Stop forward motion
            twist_msg.linear.x = 0.0
            # Turn based on 80/20 chance
            if random.random() < 0.80:
                twist_msg.angular.z = -self.turn_speed  # Turn RIGHT
            else:
                twist_msg.angular.z = self.turn_speed   # Turn LEFT

        elif self.robot_state == "scanning":
            # Stop forward motion and spin
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.scan_speed # Spin LEFT (positive)

        # --- 3. Publish Action ---
        # Send the command to the robot
        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    locomotion_brain = LocomotionBrain()
    try:
        rclpy.spin(locomotion_brain)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot when shutting down
        stop_msg = Twist()
        locomotion_brain.publisher_.publish(stop_msg)
        locomotion_brain.get_logger().info("Shutting down, stopping robot.")
        
        locomotion_brain.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
