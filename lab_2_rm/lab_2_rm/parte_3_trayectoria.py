import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
from geometry_msgs.msg import Vector3
from .controlador_PID import PController
from geometry_msgs.msg import Twist, Pose



class SigueTrayectoria(Node):
    def __init__(self):
        super().__init__("Benito")

        # publicador
        self.publisher = self.create_publisher(Float32MultiArray, "sigue_trayectoria", 10)
        self.publisher_velocidad = self.create_publisher(Vector3, "/cmd_vel_mux/input/navigation", 10)
        self.pid = PController(0.5)

        self.subscription = self.create_subscription(Float32MultiArray, '/sigue_trayectoria', self.procesar_trayectorias, 10)
        self.subscription_real_pose = self.create_subscription(Pose, '/real_pose', self.real_pose_callback, 1)

        self.posiciones_trayectoria = []
        self.posicion_actual = [1.0, 1.0]

    def procesar_trayectorias(self, list_msg):
        list_msg = list_msg.data


        #Las coordenas no llegan de manera agrupada pero si llegan en orden, por lo que hay que agruparlas en grupos de a 2 para tener todas las coordenadas
        for i in range(int(len(list_msg) / 2)):
            obj1 = list_msg.pop(0)
            obj2 = list_msg.pop(0)
            lista = [obj1, obj2]
            self.posiciones_trayectoria.append(lista)

        print(self.posiciones_trayectoria)


    def real_pose_callback(self, msg):
        x, y , or_z = msg.position.x, msg.position.y, msg.orientation.z
        print(x, y , or_z)
        # self.posicion_actual = [msg.position.x, msg.position.y]





def main(args=None):
    rclpy.init(args=args)
    node = SigueTrayectoria()


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()