import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class SphereColorOrchestrator(Node):
    def __init__(self):
        super().__init__('sphere_color_orchestrator')
        # Parámetros de color
        self.declare_parameter(
            'sphere_colors',
            ['azul', 'azul', 'naranja', 'naranja', 'amarilla']
        )
        self.sphere_colors = (
            self.get_parameter('sphere_colors')
                .get_parameter_value()
                .string_array_value
        )
        self.N = len(self.sphere_colors)

        # Crear un cliente Trigger para cada /sphere{i}/set_color
        self.__clients = []
        for i in range(self.N):
            svc_name = f'/sphere{i}/set_color'
            client = self.create_client(Trigger, svc_name)
            self.__clients.append(client)
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'Service {svc_name} no disponible')

        # Suscribirse a los sensores de proximidad
        for i in range(self.N):
            topic = f'/proximity_detection_{i}'
            self.create_subscription(
                Bool, topic,
                lambda msg, idx=i: self.handle_detection(msg, idx),
                10
            )
            self.get_logger().info(f'Subscribed to {topic}')

    def handle_detection(self, msg: Bool, idx: int):
        if not msg.data:
            return
        # Llamada asíncrona al servicio
        req = Trigger.Request()
        future = self._clients[idx].call_async(req)
        future.add_done_callback(lambda f, i=idx: self.on_response(f, i))

    def on_response(self, future, idx: int):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(
                    f'Sphere {idx} colored → {self.sphere_colors[idx]}'
                )
            else:
                self.get_logger().warn(
                    f'Error coloring sphere {idx}: {res.message}'
                )
        except Exception as e:
            self.get_logger().error(
                f'Service call failed for sphere {idx}: {e}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = SphereColorOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
