import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.listener_callback,
            10)
        self.subscription  # evita la advertencia de variable no usada
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convierte el mensaje ROS a una imagen OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error('Error al convertir imagen: %s' % str(e))
            return

        # OpenCV utiliza el formato BGR, por lo que se convierte:
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        
        # Invierte la imagen verticalmente (usa 0 para flip vertical)
        cv_image = cv2.flip(cv_image, 0)
        
        # Para invertir horizontalmente, reemplaza 0 por 1.
        # Para invertir tanto vertical como horizontalmente, usar -1.
        
        # Muestra la imagen en una ventana
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)  # Necesario para actualizar la ventana

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
