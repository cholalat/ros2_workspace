#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32MultiArray, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import math
import time
import threading
import matplotlib.pyplot as plt
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3
import numpy as np





class PController:
    def __init__(self, kp, ki = 0.0, vel_max = 0.2):
        self.vel_max = vel_max
        self.kp = kp
        self.ki = ki
        self.integral = 0.0
        self.tiempo_prev = time.time()
        
    def controlador(self, error):
        dt = 0.1

        # Parte proporcional
        p_actuation = self.kp * error

        # Parte integral
        if self.ki != 0.0:
            self.integral += error * dt
            i_actuation = self.ki * self.integral
        else: 
            i_actuation = 0.0
        
        actuation = p_actuation + i_actuation

        if actuation > self.vel_max:
            actuation = self.vel_max
        elif actuation < -self.vel_max:
            actuation = -self.vel_max
        
        return actuation






class Moverse(Node):
    def __init__(self):
        super().__init__('moverse_node')
        self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.subscription = self.create_subscription(Vector3, '/distancia_paredes', self.moverse_cb, 10)

        self.controlador_angular = PController(kp= 0.5, ki= 0.001, vel_max= 1.0)


    def moverse_cb(self, msg):
        # Obtener los valores de distancia de los obstáculos
        dis_izquierda = msg.x
        dis_centro = msg.y
        dis_derecha = msg.z

        # Calcular el error para el controlador lineal y angular
        error_angular = (dis_izquierda - dis_derecha)

        # Calcular las velocidades lineales y angulares
        vel_angular = self.controlador_angular.controlador(error_angular)

        # Crear el mensaje de velocidad y publicarlo
        msg_vel = Twist()
        if dis_centro < 0.5:
            # Si el robot está demasiado cerca de un obstáculo, detenerse
            msg_vel.linear.x = 0.0
        else:
            msg_vel.linear.x = 0.2
        msg_vel.angular.z = vel_angular
        self.publisher.publish(msg_vel)
        print(error_angular)






class DeadReckoningNav(Node):
    def __init__(self):
        super().__init__('dead_reckoning_nav_node')



        # Estado actual del turtlebot
        self.posicion_actual = [0.0, 0.0, 0.0]
        self.desplazamiento_actual = None
        self.control_activado = False
        self.desplazamiento_prev = None
        self.tiempo_actual = time.time()
        self.i = 0
        self.lista_desplazamientos = []

        # Publicadores y subscriptores
        self.state = self.create_subscription(Odometry, '/odom', self.odometry_cb, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.subscription = self.create_subscription(
            Float32MultiArray, 'goal_list', self.accion_mover_cb, 4)
        
        # Controladores linealeas y angulares
        self.controlador_lineal = PController(kp= 0.3,  ki= 0.0)
        self.controlador_angular = PController(kp= 0.3, ki= 0.0, vel_max= 1.0)

        # Timer
        self.control_timer = self.create_timer(0.1, self.control_loop)


    # Callback para la lectura de la odometria
    def odometry_cb(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion((odom.pose.pose.orientation.x,
                                                  odom.pose.pose.orientation.y,
                                                  odom.pose.pose.orientation.z,
                                                  odom.pose.pose.orientation.w,))
        
        self.posicion_actual[0] = x
        self.posicion_actual[1] = y
        self.posicion_actual[2] = yaw
        


        
    def accion_mover_cb(self, list_msg): #puntos
        lista_coordenadas = []
        list_msg = list_msg.data
        lock = threading.Lock()

        for i in range(int(len(list_msg)/3)):
            obj1 = list_msg.pop(0)
            obj2 = list_msg.pop(0)
            obj3 = list_msg.pop(0)
            lista = [obj1, obj2, obj3]
            lista_coordenadas.append(lista)


        lock.acquire()
        for coordenada in lista_coordenadas:
            self.mover_robot_a_destino(coordenada)
        lock.release()
    



    def mover_robot_a_destino(self, goal_pose): #Angulos y velocidades para llegar al punto
        self.get_logger().info(f"Entré a mover_robot")
        self.posicion_deseada = goal_pose

        d_horizontal = goal_pose[0] - self.posicion_actual[0]
        d_vertical = goal_pose[1] - self.posicion_actual[1]
        d_angular = (math.radians(goal_pose[2]) - self.posicion_actual[2] + math.pi) % (2 * math.pi) - math.pi

        desplazamiento = []
        
        desplazamiento.append((d_horizontal, 0))
        desplazamiento.append((0, d_angular))
        desplazamiento.append((d_vertical, 0))
        
        self.lista_desplazamientos.append(desplazamiento)
        if not self.control_activado:
            self.desplazamiento_actual = self.lista_desplazamientos.pop(0)
            self.control_activado = True


    def aplicar_velocidad(self, desplazamiento): #mandar velocidades y angulos al robot
        error = 0.0
        if self.i == 0:
            error = desplazamiento[0] - self.posicion_actual[0]
            vel = self.controlador_lineal.controlador(error)
            msg_vel = Twist()
            msg_vel.linear.x = vel
            self.publisher.publish(msg_vel)

        if self.i == 1:
            angulo_actual = (self.posicion_actual[2] + math.pi) % (2 * math.pi) - math.pi
            error = desplazamiento[1] - angulo_actual
            vel = self.controlador_angular.controlador(error)
            msg_vel = Twist()
            msg_vel.angular.z = vel
            self.publisher.publish(msg_vel)

        if self.i == 2:
            error = desplazamiento[0] - self.posicion_actual[1]
            vel= self.controlador_lineal.controlador(error)
            msg_vel = Twist()
            msg_vel.linear.x = vel
            self.publisher.publish(msg_vel)

        self.get_logger().info(f"Mi posicion actual es: {self.posicion_actual}")

        if abs(error) < 0.01:
            self.get_logger().info(f"Desplazamiento completado: {self.posicion_actual}")
            return True
        else:
            return False

    def control_loop(self):
        if not self.control_activado or len(self.desplazamiento_actual) == 0:
            return
        
        desplazamiento_completado = self.aplicar_velocidad(self.desplazamiento_actual[0])

        if desplazamiento_completado:
            vel_zero = Twist()
            self.publisher.publish(vel_zero)
            # Reset de los controladores
            self.controlador_lineal.integral = 0.0
            self.controlador_angular.integral = 0.0

            if len(self.desplazamiento_actual) > 1:
                self.get_logger().info(f"Estamos en la iteracion : {self.i}")
                self.desplazamiento_prev = self.desplazamiento_actual.pop(0)
                self.i += 1
            else:
                self.i = 0 
                if len(self.lista_desplazamientos) > 0:
                    self.desplazamiento_actual = self.lista_desplazamientos.pop(0)
                else:
                    self.control_activado = False
                    self.lista_desplazamientos = []











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




    node = Moverse()
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