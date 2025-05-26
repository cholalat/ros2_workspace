import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class MyNode(Node):
    def __init__(self):
        super().__init__('ricardo' + str(int(time.time() * 10000)))

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
        if self.publisher.get_subscription_count() > 0 and self.n_conectados == 0:

            informacion = '/home/paulo/universidad/5to_semestre/robotica_movil/ros2_ws/src/lab_2_rm/lab_2_rm/trayectorias/path_line.txt'

            valor = input("Qué recorrido quiere realizar? La trayectoria default es una linea recta: ")

            if valor == "sin":
                informacion = '/home/paulo/universidad/5to_semestre/robotica_movil/ros2_ws/src/lab_2_rm/lab_2_rm/trayectorias/path_sin.txt'
            elif valor == "sqrt":
                informacion = '/home/paulo/universidad/5to_semestre/robotica_movil/ros2_ws/src/lab_2_rm/lab_2_rm/trayectorias/path_sqrt.txt'



            with open(informacion, 'r') as file:
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
                self.get_logger().info('Publishing: "%s"' % msg.data)
                self.n_conectados = 1

            




def main(args=None):
    rclpy.init(args=args)
    node = MyNode()


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


