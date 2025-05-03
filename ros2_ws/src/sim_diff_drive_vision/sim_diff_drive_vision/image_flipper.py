#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

class ImageFlipper(Node):
    def __init__(self):
        super().__init__('image_flipper')
        # CvBridge for Image <-> OpenCV conversion
        self.bridge = CvBridge()

        # Publisher for flipped images
        self.publisher_ = self.create_publisher(Image, '/camera/image_flipped', 10)

        # Subscriber to original camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.listener_callback,
            10
        )
    
    def listener_callback(self, msg: Image):
        try:
            # Convert ROS message to OpenCV image (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting to CV image: {e}.')
            return
            
        # Flip vertically (top ↔ bottom)
        flipped_image = cv2.flip(cv_image, 0)
        
        try:
            # Convert back to sensor_msgs/Image
            flipped_msg = self.bridge.cv2_to_imgmsg(flipped_image, encoding='bgr8')
            # Maintain frame_id and update timestamp
            flipped_msg.header.frame_id = msg.header.frame_id
            flipped_msg.header.stamp = self.get_clock().now().to_msg()
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting to ROS Image: {e}.')
            return
            
        # Publish the flipped image
        self.publisher_.publish(flipped_msg)
        self.get_logger().debug('Image flipped and republished on /camera/image_flipped.')

def main(args=None):
    rclpy.init(args=args)

    try:
        node = ImageFlipper()
    except Exception as e:
        print(f"[FATAL] ImageFlipper failed to initialize: {e}.", file=sys.stderr)
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