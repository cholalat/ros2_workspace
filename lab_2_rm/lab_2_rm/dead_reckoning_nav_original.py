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

class DeadReckoningNav(Node):
    def __init__(self):
        super().__init__('dead_reckoning_nav_node')



        # Estado actual del turtlebot
        self.posicion_actual = [0.0, 0.0, 0.0]
        self.desplazamiento_actual = None
        self.control_activado = False
        self.desplazamiento_prev = None
        self.i = 0
        self.lista_desplazamientos = []

        # Publicadores y subscriptores
        self.state = self.create_subscription(Odometry, '/odom', self.odometry_cb, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.subscription = self.create_subscription(
            Float32MultiArray, 'goal_list', self.accion_mover_cb, 4)
        self.pos_real = self.pose_real = self.create_subscription(
            Pose, '/real_pose', self.registrar_pos_real, 10)
        
        # Controladores linealeas y angulares
        self.controlador_lineal = PController(kp= 0.3,  ki= 0.02)
        self.controlador_angular = PController(kp= 0.1, ki= 0.007, vel_max= 1.0)

        # P: kp_lineal = 0.5, kp_angular = 0.3
        # PI: kp_lineal = 0.3; ki_lineal= 0.02; kp_angular = 0.1; ki_angular = 0.007
        

        # Timer
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Graficador

        self.datos_odometro = []
        self.datos_pos_real = []
        self.tiempos = [0.0]
        self.posicion_ref_lineal = []
        self.posicion_real_lineal = []
        self.posicion_ref_angular = []
        self.posicion_real_angular = []
        self.velocidad_lineal = []
        self.velocidad_angular = []

    # Callback para la lectura de la odometria
    def odometry_cb(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion((odom.pose.pose.orientation.x,
                                                  odom.pose.pose.orientation.y,
                                                  odom.pose.pose.orientation.z,
                                                  odom.pose.pose.orientation.w,))
        
        self.posicion_actual = [x, y, yaw]
        self.datos_odometro.append([x, y])
        tiempo = self.tiempos[-1] + 0.1
        self.tiempos.append(tiempo) 
    
    def registrar_pos_real(self, msg):
        datos = msg.position
        datos = [datos.x, datos.y]
        self.datos_pos_real.append(datos)

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

                
            
    
    def mover_robot_a_destino(self, goal_pose): #Angulos y velocidades para llegar al punto

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
            if abs(self.posicion_actual[2]) > math.pi/2 and error < 0:
                vel = abs(vel)
            msg_vel = Twist()
            msg_vel.linear.x = vel
            self.publisher.publish(msg_vel)
            self.posicion_ref_lineal.append([self.tiempos[-1], desplazamiento[0]])
            self.posicion_real_lineal.append([self.tiempos[-1], self.posicion_actual[0]])
            self.velocidad_lineal.append([self.tiempos[-1], vel])

        if self.i == 1:
            angulo_actual = self.posicion_actual[2]
            error = desplazamiento[1] - angulo_actual
            vel = self.controlador_angular.controlador(error)
            msg_vel = Twist()
            msg_vel.angular.z = vel
            self.publisher.publish(msg_vel)
            self.posicion_ref_angular.append([self.tiempos[-1], desplazamiento[1]])
            self.posicion_real_angular.append([self.tiempos[-1], self.posicion_actual[2]])
            self.velocidad_angular.append([self.tiempos[-1], vel])

        if self.i == 2:
            error = desplazamiento[0] - self.posicion_actual[1]
            vel= self.controlador_lineal.controlador(error)
            if self.posicion_actual[2] < 0 and error < 0:
                vel = abs(vel)
            msg_vel = Twist()
            msg_vel.linear.x = vel
            self.publisher.publish(msg_vel)
            self.posicion_ref_lineal.append([self.tiempos[-1], desplazamiento[0]])
            self.posicion_real_lineal.append([self.tiempos[-1], self.posicion_actual[1]])
            self.velocidad_lineal.append([self.tiempos[-1], vel])

        self.get_logger().info(f"Mi posicion actual es: {self.posicion_actual}")

        if abs(round(error, 3)) < 0.01:
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
                    d_actual = self.lista_desplazamientos.pop(0)
                    self.desplazamiento_actual = d_actual
                else:
                    self.control_activado = False
                    self.lista_desplazamientos = []
                    self.desplazamiento_actual = None
                    graf = Graficador(self.datos_odometro, self.datos_pos_real, self.tiempos, self.posicion_ref_lineal, self.posicion_real_lineal,
                                      self.posicion_ref_angular, self.posicion_real_angular, self.velocidad_lineal, self.velocidad_angular)
                    graf.graficar2()


class Graficador:
    def __init__(self, pos_odom, pos_real, tiempos, pos_ref_lineal, pos_real_lineal, pos_ref_angular , pos_real_angular, 
                 vel_lineal, vel_angular):
        
        self.pos_odom = pos_odom
        self.pos_real = pos_real
        self.tiempos = tiempos
        self.pos_ref_lineal = pos_ref_lineal
        self.pos_real_lineal = pos_real_lineal
        self.pos_ref_angular = pos_ref_angular
        self.pos_real_angular = pos_real_angular
        self.vel_lineal = vel_lineal
        self.vel_angular = vel_angular



    def graficar(self):
        print("Graficando datos:")


        largo_odom = len(self.pos_odom)
        largo_real = len(self.pos_real)

        print(largo_odom, largo_real)

        l1_o = self.pos_odom

        l1_r = self.pos_real

        plt.figure(figsize=(16, 8))  # Ancho más grande para dos gráficos


        plt.subplot(1, 2, 1)
        plt.title("Coordenadas robot Odometría")
        plt.xlabel("Eje X")
        plt.ylabel("Eje Y")
        plt.scatter(*zip(*l1_o), color='red', label='vuelta 1', marker='o')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.7)


        plt.subplot(1, 2, 2)
        plt.title("Coordenadas robot Reales")
        plt.xlabel("Eje X")
        plt.ylabel("Eje Y")
        plt.scatter(*zip(*l1_r), color='red', label='vuelta 1', marker='o')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.7)

        plt.tight_layout()
        plt.show()
    
    def graficar2(self):
        
        p_ref_l = self.pos_ref_lineal
        p_real_l = self.pos_real_lineal
        p_ref_a = self.pos_ref_angular
        p_real_a = self.pos_real_angular
        vs_l = self.vel_lineal
        vs_a = self.vel_angular

        plt.figure(figsize=(16, 8))  # Ancho más grande para dos gráficos

        # --- Primer subplot (Posición + Velocidad) ---
        ax1 = plt.subplot(1, 2, 1)
        ax1.set_title("Posición y Velocidad Lineal")
        ax1.set_xlabel("Tiempo [s]")
        ax1.set_ylabel("Posición [m]")

        # Graficar posición en ax1
        ax1.plot(*zip(*p_ref_l), color='red', label='p_ref_lineal')
        ax1.plot(*zip(*p_real_l), color='green', label='p_real_lineal')

        # Segundo eje Y (velocidad)
        ax2 = ax1.twinx()
        ax2.set_ylabel("Velocidad [m/s]")
        ax2.plot(*zip(*vs_l), color='blue', label='vs_lineal')

        # Leyenda unificada
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

        ax1.grid(True, linestyle='--', alpha=0.7)


        # --- Configuración del segundo subplot (angular) ---
        ax3 = plt.subplot(1, 2, 2)  # ax3 para posición angular
        ax3.set_title("Posición y Velocidad Angular")
        ax3.set_xlabel("Tiempo [s]")
        ax3.set_ylabel("Posición Angular [rad]")

        # Gráficos de posición angular (en ax3)
        ax3.plot(*zip(*p_ref_a), color='red', label='p_ref_angular')
        ax3.plot(*zip(*p_real_a), color='green', label='p_real_angular')

        # Crear eje secundario (ax4) para velocidad angular
        ax4 = ax3.twinx()  # ax4 comparte el eje x con ax3
        ax4.set_ylabel("Velocidad Angular [rad/s]")
        ax4.plot(*zip(*vs_a), color='blue', label='vs_angular')

        # Leyenda combinada
        lines_angular = ax3.get_legend_handles_labels()
        lines_vel_angular = ax4.get_legend_handles_labels()
        ax3.legend(lines_angular[0] + lines_vel_angular[0], 
                lines_angular[1] + lines_vel_angular[1], 
                loc='upper right')

        ax3.grid(True, linestyle='--', alpha=0.7)
        plt.tight_layout()
        plt.show()
        




def main(args=None):
    print("corriendo una maraton")
    rclpy.init(args=args)
    node = DeadReckoningNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
