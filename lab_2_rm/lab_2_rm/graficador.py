import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import math
import time
import threading
import matplotlib.pyplot as plt


            # informacion = '/home/paulo/universidad/5to_semestre/robotica_movil/ros2_ws/src/lab_2_rm/lab_2_rm/trayectorias/path_line.txt'

            # valor = input("Qué recorrido quiere realizar? La trayectoria default es una linea recta: ")

            # if valor == "sin":
            #     informacion = '/home/paulo/universidad/5to_semestre/robotica_movil/ros2_ws/src/lab_2_rm/lab_2_rm/trayectorias/path_sin.txt'
            # elif valor == "sqrt":
            #     informacion = '/home/paulo/universidad/5to_semestre/robotica_movil/ros2_ws/src/lab_2_rm/lab_2_rm/trayectorias/path_sqrt.txt'





class Graficador(Node):
    def __init__(self):
        super().__init__('graficador')
        self.pos_real = []
        self.pose_real = self.create_subscription(
            Pose, '/real_pose', self.registrar_pos_real, 10)
        


        self.subscription = self.create_subscription(Float32MultiArray, '/sigue_trayectoria', self.procesar_trayectorias, 10)

        self.posiciones_trayectoria = [0,0]
        # Iniciar hilo para graficar en tiempo real
        self._running = True
        self.graficar_thread = threading.Thread(target=self.graficar_en_tiempo_real, daemon=True)
        self.graficar_thread.start()

    def registrar_pos_real(self, msg):
        datos = msg.position
        datos = [datos.x, datos.y]
        self.pos_real.append(datos)

    def procesar_trayectorias(self, list_msg):
        list_msg = list_msg.data


        #Las coordenas no llegan de manera agrupada pero si llegan en orden, por lo que hay que agruparlas en grupos de a 2 para tener todas las coordenadas
        for i in range(int(len(list_msg) / 2)):
            obj1 = list_msg.pop(0)
            obj2 = list_msg.pop(0)
            lista = [obj1, obj2]
            self.posiciones_trayectoria.append(lista)

        print(self.posiciones_trayectoria)

    def graficar_en_tiempo_real(self):
        plt.ion()
        fig, ax = plt.subplots(figsize=(8, 8))
        while self._running:
            if len(self.pos_real) > 0:
                ax.clear()
                ax.set_title("Coordenadas robot Reales")
                ax.set_xlabel("Eje X")
                ax.set_ylabel("Eje Y")
                ax.grid(True, linestyle='--', alpha=0.7)

                # Graficar la trayectoria real del robot
                ax.plot([p[0] for p in self.pos_real], [p[1] for p in self.pos_real], color='blue', marker='o', label='Trayectoria Real')

                # Graficar la trayectoria objetivo si existe
                if len(self.posiciones_trayectoria) > 1:
                    xs = [p[0] for p in self.posiciones_trayectoria if isinstance(p, list) and len(p) == 2]
                    ys = [p[1] for p in self.posiciones_trayectoria if isinstance(p, list) and len(p) == 2]
                    ax.plot(xs, ys, color='red', linestyle='--', marker='x', label='Trayectoria Objetivo')

                ax.legend()

                all_x = [p[0] for p in self.pos_real]
                all_y = [p[1] for p in self.pos_real]
                min_x, max_x = min(all_x), max(all_x)
                min_y, max_y = min(all_y), max(all_y)
                ax.set_xlim(min(0, min_x), max(5, max_x))
                ax.set_ylim(min(0, min_y), max(3, max_y))

                plt.tight_layout()
                plt.pause(0.5)
            else:
                time.sleep(0.5)
        plt.ioff()
        plt.show()

    def destroy_node(self):
        self._running = False
        super().destroy_node()

def main(args=None):
    print("corriendo una maraton")
    rclpy.init(args=args)
    node = Graficador()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()