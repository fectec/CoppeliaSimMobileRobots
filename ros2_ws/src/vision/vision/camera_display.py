import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraDisplay(Node):
    """
    A ROS2 Node that subscribes to an Image topic and displays the frames 
    using OpenCV. It also applies a vertical flip to each frame before rendering.
    """

    def __init__(self):
        super().__init__('camera_display')
        
        # Create a subscription to the '/camera/image' topic of type sensor_msgs.msg.Image
        # The callback function 'listener_callback' is invoked each time a new message arrives
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            qos.qos_profile_sensor_data
        )

        # Initialize the CvBridge, which handles conversions between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

    def image_callback(self, msg):
        """
        Callback function that triggers on receiving a new Image message.
        Converts the ROS Image into an OpenCV image, flips it vertically, 
        and then displays it in a window.
        """
        try:
            # Convert the ROS Image message to an OpenCV image in RGB format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return

        # Convert from RGB to BGR, as required by OpenCV
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        
        # Flip the image vertically
        cv_image = cv2.flip(cv_image, 0)
        
        # Display the image in a window titled "Camera Image"
        cv2.imshow("Camera Image", cv_image)

        # Wait 1 millisecond to allow the window to refresh
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraDisplay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()