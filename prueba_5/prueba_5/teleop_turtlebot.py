#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from kobuki_ros_interfaces.msg import BumperEvent
from std_msgs.msg import String, Float32MultiArray
import termios, sys, tty
import time
import threading
import math
import pygame

class TeleopTBot(Node):
    def __init__(self):
        super().__init__('teleopTBot')
        
        #Velocidades constantes
        self.linear_speed = 0.2
        self.angular_speed = 1.0

        self.tiempo_actual = time.time()


        self.publisher = self.create_publisher(Twist, "/commands/velocity", 10)

        # self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)



        self.subscription = self.create_subscription(
            BumperEvent, '/events/bumper', self.bumper_callback, 10)


        #Configuración de pygame
        pygame.init()
        pygame.display.set_mode((100, 100))
        self.clock = pygame.time.Clock()
        
        # Estado actual
        self.estado_actual = Twist()
        
        self.estado_bumper = 0

        self.get_logger().info("Teleoperación iniciada.")


    def detencion(self):
        velocidad_zero = Twist()
        self.publisher.publish(velocidad_zero)
        self.estado_actual = velocidad_zero

    def bumper_callback(self, msg):
        # self.estado_bumper = msg.state

        if msg.state == 1:
            self.get_logger().info("El Turtlebot ha detectado un choque, se inhabilitan las teclas.")
            self.detencion()
            self.estado_bumper = 1
        else:
            self.get_logger().info("Bumper liberado")
            self.estado_bumper = 0

    def aplicar_velocidad(self, tecla):
        velocidad = Twist()
        if tecla == "i":
            velocidad.linear.x = self.linear_speed
            self.get_logger().info("Se ha aplicado una velocidad (%f, %f)" % (self.linear_speed, 0))
        if tecla == "j":
            velocidad.linear.x = -self.linear_speed
            self.get_logger().info("Se ha aplicado una velocidad (%f, %f)" % (-self.linear_speed, 0))
        if tecla == "a":
            velocidad.angular.z = self.angular_speed
            self.get_logger().info("Se ha aplicado una velocidad (%f, %f)" % (0, self.angular_speed))
        if tecla == "s":
            velocidad.angular.z = -self.angular_speed
            self.get_logger().info("Se ha aplicado una velocidad (%f, %f)" % (0, -self.angular_speed))
        if tecla == "q":
            velocidad.linear.x = self.linear_speed
            velocidad.angular.z = self.angular_speed
            self.get_logger().info("Se ha aplicado una velocidad (%f, %f)" % (self.linear_speed, self.angular_speed))
        if tecla == "w":
            velocidad.linear.x = self.linear_speed
            velocidad.angular.z = -self.angular_speed
            self.get_logger().info("Se ha aplicado una velocidad (%f, %f)" % (self.linear_speed, -self.angular_speed))

        if velocidad != self.estado_actual:
            self.publisher.publish(velocidad)
            print(self.estado_bumper)        

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt
                
                if self.estado_bumper == 0:
                    tecla = pygame.key.get_pressed()

                    if tecla[pygame.K_i]:
                        self.aplicar_velocidad("i")
                    if tecla[pygame.K_j]:
                        self.aplicar_velocidad("j")
                    if tecla[pygame.K_a]:
                        self.aplicar_velocidad("a")
                    if tecla[pygame.K_s]:
                        self.aplicar_velocidad("s")
                    if tecla[pygame.K_q]:
                        self.aplicar_velocidad("q")
                    if tecla[pygame.K_w]:
                        self.aplicar_velocidad("w")
                
                    self.clock.tick(20)
            
        except KeyboardInterrupt:
            pass

        finally:
            self.publisher.publish(Twist())
            pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    nodo = TeleopTBot()
    nodo.run()
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    

    