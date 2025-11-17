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

        # --- Sub-state for avoiding logic ---
        self.avoid_state_phase = "stop" # Can be "stop" or "turn"
        self.avoid_stop_start_time = None
        self.avoid_stop_duration_sec = 0.3 # Stop for 0.3 seconds

        # --- Sensor Data Variables ---
        # We store the latest sensor data here
        self.current_obstacle_distance = 99.0
        self.is_tag_visible = False

        # --- Parameters ---
        # You can tune these values!
        self.obstacle_threshold = 0.3  # meters (30cm)
        self.forward_speed = 0.1       # m/s
        self.turn_speed = 0.8          # rad/s (Increased for a "sharper" turn)
        self.scan_speed = 0.7          # rad/s (for scan)
        
        # 2*pi radians is a full circle (~6.28 rad).
        # A 150-degree scan is ~2.62 rad.
        # 2.62 rad / 0.7 rad/s = ~3.74 seconds
        self.scan_duration_sec = 4.0  # (4s * 0.7rad/s = 2.8rad, ~160 deg scan)
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
        front_distance = msg.ranges[0]

        # Log the raw front distance for tuning
        self.get_logger().info(f"Front distance: {front_distance}")

        if front_distance == float('inf'):
            # No obstacle seen, path is clear
            self.current_obstacle_distance = 99.0
        elif front_distance == float('nan'):
            # 'nan' often means the sensor is too close or saturated.
            # Treat this as a "blocked" signal to be safe.
            self.current_obstacle_distance = 0.0 # Force a "blocked" state
        else:
            # We have a valid distance reading
            self.current_obstacle_distance = front_distance

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
                # --- RESET AVOID LOGIC ---
                self.avoid_state_phase = "stop" # Reset phase to "stop"
                self.avoid_stop_start_time = self.get_clock().now()
                self.get_logger().info("--> STATE: AVOIDING (Phase: Stop)")

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
            
            if self.avoid_state_phase == "stop":
                # Phase 1: Just stop all motion to kill momentum
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                
                # Check if stop duration has passed
                if self.avoid_stop_start_time:
                    elapsed_time = (self.get_clock().now() - self.avoid_stop_start_time).nanoseconds / 1e9
                    if elapsed_time > self.avoid_stop_duration_sec:
                        # Time's up, move to "turn" phase
                        self.avoid_state_phase = "turn"
                        self.get_logger().info("--> STATE: AVOIDING (Phase: Turn)")
            
            elif self.avoid_state_phase == "turn":
                # Phase 2: Now that we've stopped, turn in place
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
