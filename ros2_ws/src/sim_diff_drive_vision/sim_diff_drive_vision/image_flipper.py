#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ImageFlipper(Node):
    def __init__(self):
        super().__init__('image_flipper')
        # CvBridge para conversión Image <-> OpenCV
        self.bridge = CvBridge()

        # Publicador en /camera/image
        self.publisher_ = self.create_publisher(Image, '/camera/image/flipped', 10)
        # Suscriptor a /camera/image/flipped
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: Image):
        try:
            # Convertir mensaje ROS a imagen OpenCV (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error convirtiendo a CV image: {e}')
            return

        # Voltear verticalmente (cabeza ↔ pies)
        flipped_image = cv2.flip(cv_image, 0)

        try:
            # Convertir de vuelta a sensor_msgs/Image
            flipped_msg = self.bridge.cv2_to_imgmsg(flipped_image, encoding='bgr8')
            # Mantener frame_id y, si quieres, actualizar timestamp:
            flipped_msg.header.frame_id = msg.header.frame_id
            flipped_msg.header.stamp = self.get_clock().now().to_msg()
        except CvBridgeError as e:
            self.get_logger().error(f'Error convirtiendo a ROS Image: {e}')
            return

        # Publicar la imagen volteada
        self.publisher_.publish(flipped_msg)
        self.get_logger().debug('Imagen volteada y republicada en /camera/image')

def main(args=None):
    rclpy.init(args=args)
    node = ImageFlipper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

