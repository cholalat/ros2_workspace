import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class MyNode(Node):
    def __init__(self):
        super().__init__('ricardo' + str(int(time.time() * 10000)))

        # Declarar y obtener el parámetro geometry_file
        self.declare_parameter('geometry_file', "/home/paulo/universidad/5to_semestre/robotica_movil/ros2_ws/src/lab_2_rm/lab_2_rm/trayectorias/path_sin.txt")
        self.informacion = self.get_parameter('geometry_file').get_parameter_value().string_value

        # publicador
        self.publisher = self.create_publisher(Float32MultiArray, "/sigue_trayectoria", 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.archivo = []
        self.i = 0
        self.n_conectados = 0

        self.timer_callback()

    def timer_callback(self, mensaje = None):
        # Verifica si hay suscriptores antes de publicar y solo publica una vez
        if self.publisher.get_subscription_count() > 1 and self.n_conectados == 0:
            try:
                with open(self.informacion, 'r') as file:
                    lineas = file.readlines()
                    for linea in lineas:
                        linea = linea.strip()
                        linea = linea.split(",")
                        linea = list(map(float, linea))
                        self.archivo += linea
                        self.i += 1

                    msg = Float32MultiArray()
                    msg.data = self.archivo
                    self.publisher.publish(msg)
                    self.n_conectados = 1
            except Exception as e:
                self.get_logger().error(f"No se pudo abrir el archivo de trayectoria: {self.informacion} - {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


