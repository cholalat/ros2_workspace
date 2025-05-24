import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Vector3
import math
import time
import threading
import matplotlib.pyplot as plt



class DeadReckoningNav_2(Node):
    def __init__(self):
        super().__init__('dead_reckoning_nav_node_2')
        self.i = 0



        #posicion
        self.pos_actual_x = 0
        self.pos_actual_y = 0
        self.orientacion_actual = 0

        self.tiempo_actual = time.time()

        self.movimiento_siguiente = []

        self.subscription_obstacle = self.create_subscription(Vector3, "/occupancy_state", self.detenerse, 10)

        self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 4)

        self.subscription = self.create_subscription(
            Float32MultiArray, 'goal_list', self.accion_mover_cb, 1)
        

        #Es la "key" que nos va a permitir ejecutar al robot
        self.permisos = True


    def aplicar_velocidad(self, speed_command_list, id=None): #mandar velocidades y angulos al robot
        msg = Twist()
        msg.linear.x = float(speed_command_list[0])
        msg.angular.z = float(speed_command_list[1])

        tiempo_rotacion = speed_command_list[2]

        if speed_command_list[1] != 0:
            tiempo_rotacion = speed_command_list[2] * 1.1

        tiempo_actual = 0

        while abs(tiempo_actual) < abs(tiempo_rotacion):


            if self.permisos == 1:
                self.publisher.publish(msg)
                tiempo_actual += 0.01
            else:
                pass
            
            time.sleep(0.01)



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



    def accion_mover_cb(self, list_msg):  # puntos

        # Ejecutar en un hilo separado
        thread = threading.Thread(target=self.procesar_coordenadas, args=(list_msg,))
        thread.start()


    def procesar_coordenadas(self, list_msg):
        lista_coordenadas = []
        list_msg = list_msg.data

        for i in range(int(len(list_msg) / 3)):
            obj1 = list_msg.pop(0)
            obj2 = list_msg.pop(0)
            obj3 = list_msg.pop(0)
            lista = [obj1, obj2, obj3]
            lista_coordenadas.append(lista)

        for coordenada in lista_coordenadas:
            self.mover_robot_a_destino(coordenada)



    def detenerse(self, msg):

        iz = msg.x
        cen = msg.y
        der = msg.z


        if iz == 1 or cen == 1 or der == 1:
            self.permisos = False
            self.detencion()

        else:
            self.permisos = True
        print(self.permisos, iz, cen, der, self.i)
        self.i += 1



    def detencion(self):
        velocidad_zero = Twist()
        self.publisher.publish(velocidad_zero)
        self.estado_actual = velocidad_zero



def main(args=None):
    print("corriendo una maraton olimpica")
    rclpy.init(args=args)




    node = DeadReckoningNav_2()


    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.spin()





    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()