#!/usr/bin/env python3
import time
import math


class PController:
    def __init__(self, kp, ki = 0.0, vel_max = 0.2):
        self.vel_max = float(vel_max)
        self.kp = float(kp)
        self.ki = float(ki)
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
            if self.integral > 4:
                self.integral = 4
            elif self.integral < -4:
                self.integral = -4
        else: 
            i_actuation = 0.0
        
        actuation = p_actuation + i_actuation

        if actuation > self.vel_max:
            actuation = self.vel_max
        elif actuation < -self.vel_max:
            actuation = -self.vel_max
        
        return actuation
    



def angulo_con_respecto_a_punto(posicion_actual, punto_arbitrario):

    dx = punto_arbitrario[0] - posicion_actual[0]
    dy = punto_arbitrario[1] - posicion_actual[1]
    angulo = math.atan2(dy, dx)
    angulo_grados = math.degrees(angulo)
    return angulo, angulo_grados




def normalize_angle(angle):
    """Normalizes the angle to be within the range of -2*pi to 2*pi."""
    while angle > 2 * math.pi:
        angle -= 4 * math.pi
    while angle < -2 * math.pi:
        angle += 4 * math.pi
    return angle