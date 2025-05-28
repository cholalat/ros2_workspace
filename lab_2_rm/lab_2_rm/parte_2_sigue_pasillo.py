#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from geometry_msgs.msg import Vector3

from .controlador_PID import PController






class Moverse(Node):
    def __init__(self):
        super().__init__('moverse_node')
        self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.subscription = self.create_subscription(Vector3, '/distancia_paredes', self.moverse_cb, 10)

        self.controlador_angular = PController(kp= 0.5, ki= 0.0, vel_max= 1.0)

        self.vel_anterior_angular = 0.0


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
        msg_vel.angular.z = vel_angular



        if dis_centro < 0.5:
            # Si el robot está demasiado cerca de un obstáculo, detenerse
            msg_vel.linear.x = 0.0





            # Si tenemos un error angular muy pequeño cuando tenemos algo al frente, confiamos de que lo ultimo que hicimos es lo correcto y se sigue usando la vel anterior.
            if abs(error_angular) <= 0.01:
                msg_vel.angular.z = 0.1
        else:
            msg_vel.linear.x = 0.2
        self.publisher.publish(msg_vel)
        self.vel_anterior_angular = vel_angular
        print(error_angular)





def main(args=None):
    print("corriendo una maraton")
    rclpy.init(args=args)

    node = Moverse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()