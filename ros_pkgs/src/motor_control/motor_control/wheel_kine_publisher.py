#!/usr/bin/env python3

import signal
import sys
import traceback

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .quadrature_encoder import Encoder
from .dagu5_kine import Rover5

class WheelKinePublisher(Node):
    def __init__(self):
        super().__init__('wheelkine_publisher')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lw_left_pin', None),
                ('lw_right_pin', None),
                ('rw_left_pin', None),
                ('rw_right_pin', None),
                ('frequency', 10)
            ])
        
        self._lw_lp=self.get_parameter('lw_left_pin').get_parameter_value().integer_value
        self._lw_rp=self.get_parameter('lw_right_pin').get_parameter_value().integer_value
        self._rw_lp=self.get_parameter('rw_left_pin').get_parameter_value().integer_value
        self._rw_rp=self.get_parameter('rw_right_pin').get_parameter_value().integer_value

        self._lw_encoder = Encoder(self._lw_lp, self._lw_rp)
        self._rw_encoder = Encoder(self._rw_lp, self._rw_rp)

        self._rover5 = Rover5()

        self._publisher = self.create_publisher(Twist, 'wheel_speeds', 10)
        self._timer_period = 1.0/float(self.get_parameter('frequency').get_parameter_value().integer_value)
        
        # Create the timer
        self.timer = self.create_timer(self._timer_period, self.timer_callback)

    def get_speeds(self):
        ticks_l = self._lw_encoder.pull_value()
        ticks_r = self._rw_encoder.pull_value()

        wl = self._rover5.ticks_to_angular_speed(ticks_l, self._timer_period)
        wr = self._rover5.ticks_to_angular_speed(ticks_r, self._timer_period)

        ul = self._rover5.get_motor_linear_speed(wl)
        ur = self._rover5.get_motor_linear_speed(wr)

        u = self._rover5.get_global_linear_speed(ur, ul)
        w = self._rover5.get_global_angular_speed(ur, ul)

        return u, w

    def send_twist_msg(self, u, w):
        twist = Twist
        twist.linear.x=u
        twist.angular.x=w
        self._publisher.publish(twist)

    def timer_callback(self):
        """
        Callback function.
        """
        u, w = self.get_speeds()
        self.send_twist_msg(u, w)

def program_exit(wheelkine_publisher):
 
    wheelkine_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    sys.exit()


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
  
    # Create the node
    wheelkine_publisher = WheelKinePublisher()
    try:
        # Spin the node so the callback function is called.
        rclpy.spin(WheelKinePublisher)
    except KeyboardInterrupt:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        pass
    except Exception as e:
        traceback.print_exception(*sys.exc_info())
    finally:
        program_exit(wheelkine_publisher)
  
if __name__ == '__main__':
  main()