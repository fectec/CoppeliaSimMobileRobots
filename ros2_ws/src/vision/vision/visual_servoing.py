#!/usr/bin/env python3

import rclpy
import math
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Pose2D

from std_srvs.srv import SetBool

# ========================
# Utility Functions
# ========================
def wrap_to_pi(theta):
    # Wrap angle to [-pi, pi]
    result = np.fmod((theta + math.pi), (2 * math.pi))
    if result < 0:
        result += 2 * math.pi
    return result - math.pi

def quaternion_to_euler(x, y, z, w):
    # Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

# ========================
# Visual Servoing Node
# ========================
class VisualServoing(Node):
    """
    Visual Servoing Node for a differential-drive robot.
    
    This node performs the following:
      - Receives camera images and converts them into an OpenCV BGR image.
      - Converts the image into HSV color space and applies a mask to extract
        pixels in a predefined green range (the sphere).
      - Applies morphological operations (erosion then dilation) to reduce noise.
      - Finds contours in the binary mask and picks the largest contour as the sphere.
      - Computes the centroid of the sphere using image moments (m10/m00 gives the
        x-coordinate of the centroid; note: m10/m00 is the moment-based centroid calculation).
      - Uses a fixed (first measured) image center for computing the horizontal pixel error.
      - Applies a deadband: if the error is within a few pixels, no correction is applied.
      - Converts the pixel error into a heading correction (delta) and applies exponential filtering.
      - Computes a waypoint (setpoint) a fixed distance ahead along the corrected heading,
        and publishes that setpoint (x and y).
      - Displays the binary mask and a debug image (with overlays) for visualization.
      - If no green object is detected for more than a threshold (using simulation time),
        the node calls a service to tell the PID controller to stop (and sends a resume command when the ball reappears).
    """

    def __init__(self):
        super().__init__('visual_servoing')
        self.bridge = CvBridge()

        # Subscribe to the camera image topic
        self.create_subscription(Image, '/camera/image', self.image_callback, qos.qos_profile_sensor_data)
        # Subscribe to odometry to get the current robot pose
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos.qos_profile_sensor_data)
        # Subscribe to simulation time (Float32 message from CoppeliaSim)
        self.create_subscription(Float32, '/simulationTime', self.sim_time_callback, qos.qos_profile_sensor_data)
        # Publisher for the setpoint as a Float32MultiArray
        self.setpoint_pub = self.create_publisher(Float32MultiArray, '/setpoint', qos.qos_profile_sensor_data)

        # Create a client for stopping/resuming the PID controller via a service
        self.pid_stop_client = self.create_client(SetBool, '/pid_stop')
        while not self.pid_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/pid_stop service not available, waiting...")

        # Store the current robot pose as a Pose2D
        self.current_pose = Pose2D()

        # Declare and load parameters for generating the setpoint
        self.declare_parameter('d', 0.5)                # Fixed lookahead distance (m) for waypoint generation
        self.declare_parameter('k_delta', 0.005)        # Gain to convert pixel error to heading adjustment (rad/pixel)
        self.declare_parameter('error_deadband', 5)     # Deadband threshold for pixel error (pixels)
        self.declare_parameter('delta_alpha', 0.8)      # Weight factor for exponential filtering (closer to 1 gives less delay)
        self.declare_parameter('ball_timeout', 1.2)     # Time (s) after which if no ball is detected, consider it lost
        self.declare_parameter('area_threshold', 100.0) # Green contour area minimal threshold

        self.d = self.get_parameter('d').get_parameter_value().double_value
        self.k_delta = self.get_parameter('k_delta').get_parameter_value().double_value
        self.error_deadband = self.get_parameter('error_deadband').get_parameter_value().double_value
        self.delta_alpha = self.get_parameter('delta_alpha').get_parameter_value().double_value
        self.ball_timeout = self.get_parameter('ball_timeout').get_parameter_value().double_value
        self.area_threshold = self.get_parameter('area_threshold').get_parameter_value().double_value

        # Smoothing for heading correction: initialize filtered_delta to None
        self.filtered_delta = None

        # Fixed image center: once set, this value won't change
        self.image_center = None

        # Simulation time variables (s)
        self.sim_time = None
        self.last_ball_time = None

        # Flag indicating if the PID stop command has been sent
        self.pid_stopped = False

        self.get_logger().info("Visual Servoing Node Started.")

    def sim_time_callback(self, msg):
        # Update simulation time from the /simulationTime topic (s)
        self.sim_time = msg.data

    def odom_callback(self, msg):
        # Update the robot's current pose using odometry data
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Convert quaternion to Euler angles to extract yaw (robot's heading)
        _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.current_pose.theta = yaw

    def call_pid_stop_service(self, stop: bool):
        # Prepare a service request for stopping (if stop==True) or resuming (if stop==False) the PID controller
        req = SetBool.Request()
        req.data = stop
        future = self.pid_stop_client.call_async(req)
        # Use asynchronous callback so that the image_callback thread is not blocked
        future.add_done_callback(lambda fut: self.pid_stop_response_callback(fut, stop))

    def pid_stop_response_callback(self, future, stop):
        try:
            result = future.result()
            self.get_logger().info(f"PID stop service call successful: {stop}")
        except Exception as e:
            self.get_logger().error("PID stop service call failed: " + str(e))

    def image_callback(self, msg):
        try:
            # Convert the ROS image message into a BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error("CvBridge Error: " + str(e))
            return

        # Convert the BGR image to HSV color space for easier color segmentation
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define the HSV range for green
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        # Create a binary mask where green colors are white (255) and the rest is black
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Define a 5x5 kernel for morphological operations
        kernel = np.ones((5, 5), np.uint8)
        # Use erosion to remove small noise
        mask = cv2.erode(mask, kernel, iterations=1)
        # Then dilate to recover the shape of significant regions
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find external contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball_detected = False
        if contours:
            # Choose the largest contour (assumed to be the green sphere)
            largest = max(contours, key=cv2.contourArea)
            # Proceed only if the area is above a minimal threshold (to avoid small false detections)
            if cv2.contourArea(largest) > self.area_threshold:
                # Calculate image moments of the largest contour
                # m10 and m01 are spatial moments; m00 is the area
                # Dividing m10 by m00 gives the x-coordinate of the centroid
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    ball_detected = True
                    # Update the last time a ball was detected using simulation time
                    self.last_ball_time = self.sim_time

                    # If the PID was stopped, call the service to resume
                    if self.pid_stopped:
                        self.call_pid_stop_service(False)
                        self.pid_stopped = False

                    # Compute the x and y coordinates of the centroid
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    # Draw a small circle at the centroid for debugging
                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

                    # Determine the fixed image center on the first measurement
                    h, w = mask.shape
                    if self.image_center is None:
                        self.image_center = w / 2.0  # Fixed center remains constant

                    # Compute the horizontal error: (centroid x - fixed image center)
                    error_x = cx - self.image_center

                    # Apply a deadband: if error is small enough, treat it as zero
                    if abs(error_x) < self.error_deadband:
                        error_x = 0

                    # Calculate the raw heading correction (delta), in radians
                    computed_delta = -self.k_delta * error_x

                    # Apply exponential filtering to smooth the computed delta
                    if self.filtered_delta is None:
                        self.filtered_delta = computed_delta
                    else:
                        self.filtered_delta = (self.delta_alpha * computed_delta +
                                               (1 - self.delta_alpha) * self.filtered_delta)

                    # Compute the desired heading by adjusting the current robot heading
                    # wrap_to_pi ensures that the angle remains in the range [-pi, pi]
                    desired_heading = wrap_to_pi(self.current_pose.theta + self.filtered_delta)

                    # Compute the goal waypoint a fixed distance ahead along the desired heading
                    x_goal = self.current_pose.x + self.d * math.cos(desired_heading)
                    y_goal = self.current_pose.y + self.d * math.sin(desired_heading)

                    # Build the setpoint message containing only x and y
                    setpoint_msg = Float32MultiArray()
                    setpoint_msg.data = [x_goal, y_goal]
                    self.setpoint_pub.publish(setpoint_msg)

                    self.get_logger().info(
                        f"Setpoint published: x: {x_goal:.2f}, y: {y_goal:.2f}"
                    )

                    # Draw a blue line from the fixed image center to the centroid (for debug)
                    cv2.line(cv_image, (int(self.image_center), int(h / 2)), (cx, cy), (255, 0, 0), 2)
                    # Overlay the computed delta (filtered) on the image for debug
                    cv2.putText(cv_image, f"Delta: {self.filtered_delta:.3f} rad", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        # If no ball is detected, check the elapsed simulation time since the last detection
        if not ball_detected:
            if self.last_ball_time is not None and self.sim_time is not None:
                elapsed = self.sim_time - self.last_ball_time
                # If elapsed simulation time exceeds ball_timeout and the PID has not already been stopped, then call the service
                if elapsed > self.ball_timeout and not self.pid_stopped:
                    self.call_pid_stop_service(True)
                    self.pid_stopped = True
                    # Reset the filtered delta
                    self.filtered_delta = None
                    self.get_logger().info("No green object detected, stopping the car via PID stop service.")

        # Display the binary mask (for visual debugging of the green areas)
        cv2.imshow("Green Mask", mask)
        # Display the camera image with debug overlays (centroid, line, text)
        cv2.imshow("Camera Debug", cv_image)
        # Wait 1 millisecond to refresh the image windows
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Visual Servoing Node interrupted. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
