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



class DeadReckoningNav(Node):
    def __init__(self):
        super().__init__('dead_reckoning_nav_node')
        self.i = 0



        #posicion
        self.pos_actual_x = 0
        self.pos_actual_y = 0
        self.orientacion_actual = 0

        self.tiempo_actual = time.time()

        self.movimiento_siguiente = []


        self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 4)

        self.subscription = self.create_subscription(
            Float32MultiArray, 'goal_list', self.accion_mover_cb, 10)
        



    def aplicar_velocidad(self, speed_command_list, id=None): #mandar velocidades y angulos al robot
        msg = Twist()
        msg.linear.x = float(speed_command_list[0])
        msg.angular.z = float(speed_command_list[1])

        tiempo_rotacion = speed_command_list[2]
        self.tiempo_actual = time.time()

        if speed_command_list[1] != 0:
            tiempo_rotacion = speed_command_list[2] * 1.1

        while abs(time.time() - self.tiempo_actual) < abs(tiempo_rotacion):
            self.publisher.publish(msg)


        self.orientacion_actual += float(speed_command_list[2]) * speed_command_list[1]
        self.pos_actual_x += round(float(speed_command_list[2]) * speed_command_list[0] * math.cos(self.orientacion_actual), 5)
        self.pos_actual_y += round(float(speed_command_list[2]) * speed_command_list[0] * math.sin(self.orientacion_actual), 5)
        
        detension = Twist()
        detension.linear.x = float(0)
        detension.linear.z = float(0)

        self.publisher.publish(detension)

        self.orientacion_actual % (2 * math.pi)
        if self.orientacion_actual > math.pi:
            self.orientacion_actual -= 2 * math.pi



    def mover_robot_a_destino(self, goal_pose): #Angulos y velocidades para llegar al punto
        vel_lineal = 0.2
        vel_angular = 1


        #moverme al origen
        x_visto_desde_origen = goal_pose[0] - self.pos_actual_x

        #Verificamos que no estamos ya en el punto deseado
        if round(x_visto_desde_origen, 3) != 0:

            #Orientarme con eje x
            tiempo_1 = abs(self.orientacion_actual / vel_angular)
            velocidad_1 = -vel_angular

            if x_visto_desde_origen < 0:
                velocidad_1 = -velocidad_1
                tiempo_1 = (math.pi - self.orientacion_actual) / vel_angular

            self.aplicar_velocidad([0, velocidad_1, tiempo_1], "alineando con eje x")


            #Avanzar en el eje x
            tiempo_2 = abs(x_visto_desde_origen / vel_lineal)
            self.aplicar_velocidad([vel_lineal, 0, tiempo_2], "avancé a la posicion en el eje x")



        y_visto_desde_origen = goal_pose[1] - self.pos_actual_y


        #Verificamos que no estamos ya en el punto deseado
        if round(y_visto_desde_origen, 3) != 0:
            #Orientarse con eje y
            tiempo_3 = abs(((math.pi/2) - self.orientacion_actual)  / vel_angular)
            velocidad_3 = vel_angular
            
            if y_visto_desde_origen < 0 and x_visto_desde_origen >= 0:
                velocidad_3 = -velocidad_3
                tiempo_3  = abs(((math.pi/2) + self.orientacion_actual)  / vel_angular)

            if y_visto_desde_origen > 0 and x_visto_desde_origen < 0:
                velocidad_3 = -velocidad_3
                tiempo_3  = abs(((math.pi/2) + self.orientacion_actual)  / vel_angular)


            self.aplicar_velocidad([0, velocidad_3, tiempo_3], "Alineando con eje y")

            #Avanzar en el eje y
            tiempo_4 = abs(y_visto_desde_origen / vel_lineal)
            self.aplicar_velocidad([vel_lineal, 0, tiempo_4], "Avanzando a la posicion en el eje y")


        #Corregir angulo
        orientacion_final = (math.radians(goal_pose[2]) - self.orientacion_actual) % (2 * math.pi)
        if orientacion_final > math.pi:
            orientacion_final -= 2 * math.pi

        if orientacion_final != round(0, 3):
            velocidad_5 = vel_angular
            tiempo_5 = abs(orientacion_final / vel_angular)

            if round(orientacion_final,3) < 0:
                velocidad_5 = -velocidad_5
            self.aplicar_velocidad([0, velocidad_5, tiempo_5], "Colocando angulo final pedido")






    def accion_mover_cb(self, list_msg): #puntos
        lista_coordenadas = []
        list_msg = list_msg.data

        for i in range(int(len(list_msg)/3)):
            obj1 = list_msg.pop(0)
            obj2 = list_msg.pop(0)
            obj3 = list_msg.pop(0)
            lista = [obj1, obj2, obj3]
            lista_coordenadas.append(lista)


        for coordenada in lista_coordenadas:
            self.mover_robot_a_destino(coordenada)







class Graficador(Node):
    def __init__(self):
        super().__init__('graficador')

        
        self.pos_odom = []
        self.pos_real = []

        self.odometro = self.create_subscription(
            Odometry, '/odom', self.registrar_odom, 10)
        
        self.pose_real = self.create_subscription(
            Pose, '/real_pose', self.registrar_pos_real, 10)


    def registrar_odom(self, msg):
        datos = msg.pose.pose.position
        datos = [datos.x, datos.y]
        self.pos_odom.append(datos)

    def registrar_pos_real(self, msg):
        datos = msg.position
        datos = [datos.x, datos.y]
        self.pos_real.append(datos)


    def graficar(self, msg):
        print("Graficando datos:", msg)

        if msg == "g":

            largo_odom = len(self.pos_odom)
            largo_real = len(self.pos_real)

            print(largo_odom, largo_real)

        l1_o = self.pos_odom[0 : int(largo_odom/3)]
        l2_o = self.pos_odom[int(largo_odom/3) : int(2 * largo_odom/3)]
        l3_o = self.pos_odom[int(2 * largo_odom/3) : int(largo_odom)]

        l1_r = self.pos_real[0 : int(largo_real/3)]
        l2_r = self.pos_real[int(largo_real/3) : int(2 * largo_real/3)]
        l3_r = self.pos_real[int(2 * largo_real/3) : int(largo_real)]

        plt.figure(figsize=(16, 8))  # Ancho más grande para dos gráficos


        plt.subplot(1, 2, 1)
        plt.title("Coordenadas robot Odometría")
        plt.xlabel("Eje X")
        plt.ylabel("Eje Y")
        plt.scatter(*zip(*l1_o), color='red', label='vuelta 1', marker='o')
        plt.scatter(*zip(*l2_o), color='green', label='vuelta 2', marker='s')
        plt.scatter(*zip(*l3_o), color='blue', label='vuelta 3', marker='^')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.7)


        plt.subplot(1, 2, 2)
        plt.title("Coordenadas robot Reales")
        plt.xlabel("Eje X")
        plt.ylabel("Eje Y")
        plt.scatter(*zip(*l1_r), color='red', label='vuelta 1', marker='o')
        plt.scatter(*zip(*l2_r), color='green', label='vuelta 2', marker='s')
        plt.scatter(*zip(*l3_r), color='blue', label='vuelta 3', marker='^')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.7)

        plt.tight_layout()
        plt.show()




def input_thread(node):
    while rclpy.ok():
        mensaje = input()
        if mensaje.strip():
            node.graficar(mensaje)


def main(args=None):
    print("corriendo una maraton")
    rclpy.init(args=args)




    node = DeadReckoningNav()
    node2 = Graficador()


    hilo_entrada = threading.Thread(target=input_thread, args=(node2,),
                                    daemon=True)
    hilo_entrada.start()


    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node2)
    executor.spin()





    executor.shutdown()
    node.destroy_node()
    node2.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()