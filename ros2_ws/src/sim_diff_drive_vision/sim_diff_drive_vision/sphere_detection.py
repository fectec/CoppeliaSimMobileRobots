#!/usr/bin/env python3

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import sys

import rclpy
from rclpy.node import Node
from rclpy import qos

from sensor_msgs.msg import Image
from custom_interfaces.msg import SphereDetection

class SphereDetectionNode(Node):
    def __init__(self):
        super().__init__('sphere_detection')
        self.bridge = CvBridge()

        # Publisher for our custom message
        self.sphere_detection_pub = self.create_publisher(SphereDetection, '/sphere_detection', 10)

        # Subscribe to the raw camera topic
        self.create_subscription(
            Image, 
            '/camera/image', 
            self.image_callback, 
            qos.qos_profile_sensor_data
        )

        # Minimal contour area to count as a real sphere
        self.declare_parameter('area_threshold', 200.0)
        self.area_threshold = self.get_parameter('area_threshold').value

        # Define HSV ranges for each color
        self.hsv_ranges = {
            SphereDetection.COLOR_GREEN:  (np.array([40,  70,  70]),  np.array([80, 255, 255])),
            SphereDetection.COLOR_ORANGE: (np.array([ 5, 150, 150]), np.array([20, 255, 255])),
            SphereDetection.COLOR_BLUE:   (np.array([90, 100, 100]), np.array([130, 255, 255])),
            SphereDetection.COLOR_YELLOW: (np.array([20, 150, 150]), np.array([35, 255, 255])),
        }

        self.get_logger().info("SphereDetection Start.")

    def image_callback(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}.")
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        detection = SphereDetection()
        detection.color = SphereDetection.COLOR_NONE
        detection.centroid_x = 0.0
        detection.centroid_y = 0.0

        max_area = 0.0
        
        # For each color, find largest contour
        for color, (lo, hi) in self.hsv_ranges.items():
            mask = cv2.inRange(hsv, lo, hi)
            # Clean up noise
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            # Pick the largest contour of this color
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area < self.area_threshold or area <= max_area:
                continue

            # Compute centroid
            M = cv2.moments(largest)
            if M['m00'] == 0:
                continue

            cx = float(M['m10']/M['m00'])
            cy = float(M['m01']/M['m00'])

            # Update best detection
            max_area = area
            detection.color = color
            detection.centroid_x = cx
            detection.centroid_y = cy

        # Publish whatever we found
        self.sphere_detection_pub.publish(detection)

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