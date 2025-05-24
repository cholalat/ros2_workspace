import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import sys
import time
import os

class MyNode(Node):
    def __init__(self):
        super().__init__('ricardo' + str(int(time.time() * 10000)))

        # publicador
        self.publisher = self.create_publisher(Float32MultiArray, "goal_list", 10)
        timer_period = 0.5
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.archivo = []
        self.i = 0

        self.timer_callback()


    def timer_callback(self, mensaje = None):


        informacion = '/home/paulo/universidad/5to_semestre/robotica_movil/ros2_ws_lab_1/src/prueba_5/prueba_5/coordenadas.txt'

        with open(informacion, 'r') as file:
            lineas = file.readlines()
            for linea in lineas:

                linea = linea.strip()
                linea = linea.split(",")
                linea = list(map(float, linea))

                self.archivo += linea
                self.i += 1
                time.sleep(0.1)

            msg = Float32MultiArray()
            msg.data = self.archivo
            self.publisher.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)





def main(args=None):
    rclpy.init(args=args)
    node = MyNode()


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


