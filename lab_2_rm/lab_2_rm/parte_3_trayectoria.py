import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
from geometry_msgs.msg import Vector3
from .controlador_PID import PController, angulo_con_respecto_a_punto
from geometry_msgs.msg import Twist, Pose
import math
from tf_transformations import euler_from_quaternion



class SigueTrayectoria(Node):
    def __init__(self):
        super().__init__("Benito")

        # publicador
        self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.pid = PController(1.5, 0.01, 1)

        self.subscription = self.create_subscription(Float32MultiArray, '/sigue_trayectoria', self.procesar_trayectorias, 10)
        self.subscription_real_pose = self.create_subscription(Pose, '/real_pose', self.real_pose_callback, 1)
        self.subscription_odom = self.create_subscription(Pose, '/odom', self.odom_callback, 1)

        timer_callback_period = 0.1
        self.timer = self.create_timer(timer_callback_period, self.ir_a_destino)

        self.posiciones_trayectoria = []
        self.posicion_actual = [1.0, 1.0, 0]
        self.posicion_odom = [0, 0, 0]

        self.lad = 0.05 #Aquí definimos la look ahead distance

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
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.posicion_actual = [msg.position.x, msg.position.y, yaw]


    def odom_callback(self, msg):
        self.posicion_odom = [msg.position.x, msg.position.y, msg.orientation.z]
        # self.get_logger().info(f"Posición actual: {self.posicion_actual}")


    def ir_a_destino(self):
        if len(self.posiciones_trayectoria) == 0:
            # print("No hay trayectorias a seguir")
            return
        
        punto_mas_cercano = self.posiciones_trayectoria[0]
        dis_punto_más_cercano = 10000000000000000000000000

        copia_posiciones_trayectoria = self.posiciones_trayectoria.copy()
        ultima_distacia = 0

        for posicion in range(len(self.posiciones_trayectoria)):
            dx = self.posiciones_trayectoria[posicion][0] - self.posicion_actual[0]
            dy = self.posiciones_trayectoria[posicion][1] - self.posicion_actual[1]
            distancia_2 = dx**2 + dy**2
            if distancia_2 < dis_punto_más_cercano:
                punto_mas_cercano = [self.posiciones_trayectoria[posicion][0] , self.posiciones_trayectoria[posicion][1]]
                dis_punto_más_cercano = distancia_2

                #Esta copia la hacemos para que los puntos que use de referencia despues sean los que están por delante y no por detrás en la secuencia de trayectorias.
                copia_posiciones_trayectoria = self.posiciones_trayectoria[posicion:].copy()
            ultima_distacia = distancia_2

        #Ahora buscamos el punto más alejado de nuestro punto más cercano pero que aún se mantenga dentro del look ahead distance
        carrot = punto_mas_cercano
        dis_valor_anterior = 0
        for coordenada in copia_posiciones_trayectoria:
            dx = punto_mas_cercano[0] - coordenada[0]
            dy = punto_mas_cercano[1] - coordenada[1]
            distancia_2 = dx**2 + dy**2

            if distancia_2 >= self.lad:
                break
            elif distancia_2 > dis_valor_anterior:
                carrot = coordenada
                dis_valor_anterior = distancia_2



        theta_ref, theta_ref_degrees = angulo_con_respecto_a_punto(self.posicion_actual, carrot)


        error = theta_ref - self.posicion_actual[2]
        error = math.atan2(math.sin(error), math.cos(error)) # keep error between -pi and pi

        vel_angular = self.pid.controlador(error)



        msg_vel = Twist()
        msg_vel.linear.x = 0.1
        msg_vel.angular.z = vel_angular

        if ultima_distacia < 0.01:
            msg_vel.linear.x = 0.0
            msg_vel.angular.z = 0.0
            

        self.publisher.publish(msg_vel)

        print("angulo_actual real: ", math.degrees(self.posicion_actual[2]))
        print("angulo referencia: ", math.degrees(theta_ref))
        print("error: ", math.degrees(error))
        print("velocidad angular: ", vel_angular)




def main(args=None):
    rclpy.init(args=args)
    node = SigueTrayectoria()


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()