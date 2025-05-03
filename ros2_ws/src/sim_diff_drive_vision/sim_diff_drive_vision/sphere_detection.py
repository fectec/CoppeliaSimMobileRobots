#!/usr/bin/env python3

import math
import sys
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node
from rclpy import qos

from tf_transformations import euler_from_quaternion
from sim_diff_drive_utils.utils.math_helpers import wrap_to_pi

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from custom_interfaces.msg import SphereDetection

class SphereDetectionNode(Node):
    def __init__(self):
        super().__init__('sphere_detection')
        self.bridge = CvBridge()
        
        # Minimal contour area to count as a real sphere
        self.declare_parameter('area_threshold', 10.0)
        self.area_threshold = self.get_parameter('area_threshold').value
        
        # Declare parameters for waypoint generation
        self.declare_parameter('d', 0.5)                # Fixed lookahead distance (m) for waypoint generation
        self.declare_parameter('k_delta', 0.005)        # Gain to convert pixel error to heading adjustment (rad/pixel)
        self.declare_parameter('error_deadband', 5)     # Deadband threshold for pixel error (pixels)
        self.declare_parameter('delta_alpha', 0.8)      # Weight factor for exponential filtering (closer to 1 gives less delay)
        
        # Debugging window parameter
        self.declare_parameter('debug_view', True)
        self.debug_view = self.get_parameter('debug_view').value
    
        self.d = self.get_parameter('d').value
        self.k_delta = self.get_parameter('k_delta').value
        self.error_deadband = self.get_parameter('error_deadband').value
        self.delta_alpha = self.get_parameter('delta_alpha').value

        # Publisher for our custom message
        self.sphere_detection_pub = self.create_publisher(SphereDetection, '/sphere_detection', 10)
        
        # Subscribe to the raw camera topic
        self.create_subscription(
            Image,
            '/camera/image_flipped',
            self.image_callback,
            qos.qos_profile_sensor_data
        )
        
        # Subscribe to odometry to get the current robot pose
        self.create_subscription(
            Odometry, 
            'sim_diff_drive/odom', 
            self.odom_callback, 
            qos.qos_profile_sensor_data
        )
        
        # Subscribe to simulation time
        self.create_subscription(
            Float32, 
            'simulationTime', 
            self.sim_time_callback, 
            qos.qos_profile_sensor_data
        )
        
        # Store the current robot pose as a Pose2D
        self.current_pose = Pose2D()
        # Flag to check if we've received odometry data
        self.odometry_received = False
        
        # Fixed image center: once set, this value won't change
        self.image_center = None
        
        # Smoothing for heading correction: initialize filtered_delta to None
        self.filtered_delta = None
        
        # Time tracking
        self.now_time = None
        self.time_received = False
        
        # Define HSV ranges for each color
        self.hsv_ranges = {
            SphereDetection.COLOR_GREEN: (np.array([40, 70, 70]), np.array([80, 255, 255])),
            SphereDetection.COLOR_ORANGE: (np.array([ 5, 150, 150]), np.array([20, 255, 255])),
            SphereDetection.COLOR_BLUE: (np.array([90, 100, 100]), np.array([130, 255, 255])),
            SphereDetection.COLOR_YELLOW: (np.array([20, 150, 150]), np.array([35, 255, 255])),
        }
        
        # Color name mapping for debug visualization
        self.color_names = {
            SphereDetection.COLOR_GREEN: "GREEN",
            SphereDetection.COLOR_ORANGE: "ORANGE",
            SphereDetection.COLOR_BLUE: "BLUE",
            SphereDetection.COLOR_YELLOW: "YELLOW",
            SphereDetection.COLOR_NONE: "NONE"
        }
        
        # BGR color values for visualization
        self.bgr_colors = {
            SphereDetection.COLOR_GREEN: (0, 255, 0),    # Green
            SphereDetection.COLOR_ORANGE: (0, 165, 255), # Orange
            SphereDetection.COLOR_BLUE: (255, 0, 0),     # Blue
            SphereDetection.COLOR_YELLOW: (0, 255, 255), # Yellow
            SphereDetection.COLOR_NONE: (255, 255, 255)  # White
        }
        
        if self.debug_view:
            # Create window without decorations to eliminate white margins
            cv2.namedWindow('Sphere Detection Debug', cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL | cv2.WINDOW_FREERATIO)
            cv2.resizeWindow('Sphere Detection Debug', 320, 240)  # Smaller window size
        
        self.get_logger().info("SphereDetection Start. Debug view: {}".format(
            "Enabled" if self.debug_view else "Disabled"))
    
    def sim_time_callback(self, msg):
        # Update simulation time from the /simulationTime topic (s)
        self.now_time = msg.data
        self.time_received = True
    
    def odom_callback(self, msg):
        # Update x, y
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y

        # Convert the incoming orientation quaternion to Euler angles,
        # then store the yaw component
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose.theta = yaw
        self.odometry_received = True

    def image_callback(self, msg):
        # Check if we have received necessary data
        if not self.odometry_received or not self.time_received:
            self.get_logger().debug("Waiting for odometry and time data...")
            return
            
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}.")
            return
            
        # Create a copy for visualization
        debug_img = bgr.copy()
        
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        detection = SphereDetection()
        detection.color = SphereDetection.COLOR_NONE
        detection.x = 0.0
        detection.y = 0.0
        
        # For debugging: store all contours and areas by color
        all_contours = {}
        valid_detections = {}
        
        # First pass: find all valid contours for each color
        for color, (lo, hi) in self.hsv_ranges.items():
            mask = cv2.inRange(hsv, lo, hi)
            
            # Clean up noise
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                all_contours[color] = []
                continue
                
            # Store all contours for this color
            all_contours[color] = contours
            
            # Pick the largest contour of this color
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            # Only consider contours above the threshold
            if area < self.area_threshold:
                continue
                
            # Compute centroid
            M = cv2.moments(largest)
            if M['m00'] == 0:
                continue
                
            cx = float(M['m10']/M['m00'])
            cy = float(M['m01']/M['m00'])
            
            # Store as a valid detection
            valid_detections[color] = {
                'area': area,
                'cx': cx,
                'cy': cy,
                'contour': largest
            }
            
        # Determine which color's detection we will publish
        max_area = 0.0
        pub_color = SphereDetection.COLOR_NONE
        
        # Check for green first (still give it priority for the published color)
        if SphereDetection.COLOR_GREEN in valid_detections:
            pub_color = SphereDetection.COLOR_GREEN
            max_area = valid_detections[pub_color]['area']
        # If no green, find the color with the largest area
        elif valid_detections:
            for color, info in valid_detections.items():
                if info['area'] > max_area:
                    max_area = info['area']
                    pub_color = color
        else:
            # No valid detection
            detection.color = SphereDetection.COLOR_NONE
            detection.x = 0.0
            detection.y = 0.0
            self.sphere_detection_pub.publish(detection)
            
            # Debug visualization for no detection
            if self.debug_view:
                cv2.putText(debug_img, "NO SPHERE", (5, 15), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.imshow('Sphere Detection Debug', debug_img)
                cv2.waitKey(1)
            return
        
        # Get the centroid of the detected color
        cx = valid_detections[pub_color]['cx']
        cy = valid_detections[pub_color]['cy']
        
        # Always use green centroid for waypoint calculation, if available
        # If green is detected at all, use its centroid for x_goal, y_goal
        waypoint_color = SphereDetection.COLOR_GREEN if SphereDetection.COLOR_GREEN in valid_detections else pub_color
        waypoint_cx = valid_detections[waypoint_color]['cx']
        waypoint_cy = valid_detections[waypoint_color]['cy']
        
        # Calculate waypoint coordinates using the algorithm
        # Determine the fixed image center on the first measurement
        h, w = bgr.shape[:2]
        if self.image_center is None:
            self.image_center = w / 2.0  # Fixed center remains constant

        # Compute the horizontal error: (centroid x - fixed image center)
        # Using the waypoint_color's centroid (which is green if available)
        error_x = waypoint_cx - self.image_center

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
        
        # Create the detection with the published color but waypoint coordinates
        detection.color = pub_color
        detection.x = x_goal
        detection.y = y_goal
        
        # Publish the detection
        self.sphere_detection_pub.publish(detection)
        
        # Debug logging
        self.get_logger().debug(
            f"Published: color={self.color_names[pub_color]}, x={x_goal:.2f}, y={y_goal:.2f}"
        )
        
        # Debug visualization
        if self.debug_view:
            # Draw all contours in faded colors
            for color, contours in all_contours.items():
                bgr_color = self.bgr_colors[color]
                # Draw with half opacity for non-detected contours
                cv2.drawContours(debug_img, contours, -1, bgr_color, 1)
                
            # Draw the detected sphere's contour
            largest_contour = max(all_contours[pub_color], key=cv2.contourArea)
            cv2.drawContours(debug_img, [largest_contour], -1, self.bgr_colors[pub_color], 3)
            
            # Draw the original centroid
            orig_centroid = (int(cx), int(cy))
            cv2.circle(debug_img, orig_centroid, 7, self.bgr_colors[pub_color], -1)
            
            # Also mark the waypoint centroid if it's different from the published color
            if waypoint_color != pub_color:
                waypoint_centroid = (int(waypoint_cx), int(waypoint_cy))
                cv2.circle(debug_img, waypoint_centroid, 7, self.bgr_colors[waypoint_color], -1)
                cv2.putText(debug_img, "WP", (waypoint_centroid[0] + 10, waypoint_centroid[1]), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.bgr_colors[waypoint_color], 1)
            
            # Draw crosshairs at the centroid
            cv2.line(debug_img, 
                     (orig_centroid[0] - 15, orig_centroid[1]), 
                     (orig_centroid[0] + 15, orig_centroid[1]), 
                     self.bgr_colors[pub_color], 2)
            cv2.line(debug_img, 
                     (orig_centroid[0], orig_centroid[1] - 15), 
                     (orig_centroid[0], orig_centroid[1] + 15), 
                     self.bgr_colors[pub_color], 2)
            
            # Add text information
            text = f"{self.color_names[pub_color]} Delta: {self.filtered_delta:.3f}"
            cv2.putText(debug_img, text, (5, 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.bgr_colors[pub_color], 1)
            
            # Show the image
            cv2.imshow('Sphere Detection Debug', debug_img)
            cv2.waitKey(1)  # Process events, wait 1ms

    def destroy_node(self):
        if self.debug_view:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SphereDetectionNode()
    except Exception as e:
        print(f"[FATAL] SphereDetection failed to initialize: {e}.", file=sys.stderr)
        rclpy.shutdown()
        return
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted with Ctrl+C.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()