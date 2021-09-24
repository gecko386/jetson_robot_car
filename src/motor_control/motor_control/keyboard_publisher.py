#!/usr/bin/env python3

#ROS2 packages
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive

#keyboard packages
import pynput
from pynput import keyboard

#math packages
import math

#increment definitions:

SPEED_STEPS = 0.01
ANGLE_STEPS = (math.pi/(2.0*100.0))


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')

        #Ackermann orders
        self._last_angle = 0.0
        self._last_speed = 0.0
        self._need_update_ackermann = True

        self._publisher = self.create_publisher(AckermannDrive, 'ackermann_input', 10)
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #keyboard management
        self._key_buffer = []
        self._key_buffer = []
        self._key_listener = keyboard.Listener(
            on_press=self._on_press)

        self._ignore_events = False

        self._key_listener.start()

    def _on_press(self, key):
        if not self._ignore_events:
            self._key_buffer.append(key)

    def clamp_to_one(self, value, limit = 1.0):
        if value > limit:
            return limit
        elif value < -limit:
            return -limit
        else:
            return value

    def process_key_arrows(self, k):
        if k == keyboard.Key.up:
            val = self.clamp_to_one(self._last_speed + SPEED_STEPS)
            if self._last_speed == val:
                return
            
            self._last_speed = val
            self._need_update_ackermann = True

        elif k == keyboard.Key.down:
            val = self.clamp_to_one(self._last_speed - SPEED_STEPS)
            if self._last_speed == val:
                return
            
            self._last_speed = val
            self._need_update_ackermann = True

        elif k == keyboard.Key.right:
            val = self.clamp_to_one(self._last_angle + ANGLE_STEPS)
            if self._last_angle == val:
                return
            
            self._last_angle = val
            self._need_update_ackermann = True

        elif k == keyboard.Key.left:
            val = self.clamp_to_one(self._last_angle - ANGLE_STEPS)
            if self._last_angle == val:
                return
            
            self._last_angle = val
            self._need_update_ackermann = True

    def process_events(self):
        self._ignore_events = True
        keys = self._key_buffer.copy()
        self._key_buffer = []
        self._ignore_events = False
        for k in keys:
            self.process_key_arrows(k)
            
    def send_ackermann_msg(self):
        if self._need_update_ackermann:
            order = AckermannDrive()
            order.steering_angle = self._last_angle
            order.speed = self._last_speed
            self._publisher.publish(order)
            self.get_logger().info('Publishing Ackermann Msg:' + str(order))
            self._need_update_ackermann = False

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        self.process_events()        
        self.send_ackermann_msg()

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
  
    # Create the node
    keyboard_publisher = KeyboardPublisher()
  
    # Spin the node so the callback function is called.
    rclpy.spin(keyboard_publisher)
  
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_publisher.destroy_node()
  
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()