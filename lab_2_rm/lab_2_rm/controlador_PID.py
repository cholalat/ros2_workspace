#!/usr/bin/env python3
import time



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