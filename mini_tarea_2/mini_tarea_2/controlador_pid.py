#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty
import time

class PIDController(Node):
    def __init__( self, kp, ki = 0, kd = 0):
        super().__init__( 'p_controller' )
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = 0.1

        self.int = 0
        self.tiempo = time.time()


        self.error_anterior = 0

        self.setpoint = None
        self.state = None
        self.proportional_action = 0
        self.actuation_pub = self.create_publisher( Float64, 'control_effort', 1 )

        self.dist_set_point_sub = self.create_subscription( Float64, 'setpoint',
        self.setpoint_cb, 1 )

        self.dist_state_sub = self.create_subscription( Float64, 'state',
        self.state_cb, 1 )


    def setpoint_cb( self, msg ):
        self.get_logger().info( '[PICTRL] new setpoint received: %.2f' % (msg.data) )
        self.reset()
        self.setpoint = msg.data

    def state_cb( self, msg ):
        if self.setpoint == None:
            return
        self.state = msg.data


        error = self.setpoint - self.state
        de = error - self.error_anterior

        tiempo_ac = time.time()
        dt = tiempo_ac - self.tiempo
        self.tiempo = tiempo_ac




        # Proportional
        p_actuation = self.kp*error
        # Integrative (Implement me!)



        self.int += self.ki * error * dt
        # Derivative (Implement me!)



        d_actuation = self.kd * (de / dt)



        # Actuation
        actuation = p_actuation + self.int + d_actuation
        # Message sending
        msg = Float64()
        msg.data = actuation
        self.actuation_pub.publish( msg )
        print(msg.data)

    def reset( self ):
        self.setpoint = None
        self.state = None



def main():
    rclpy.init()
    p_ctrl = PIDController( 0.5, 0.1, 0.01)
    rclpy.spin( p_ctrl )

if __name__ == '__main__':
    main()