#!/usr/bin/env python3

import signal
import sys

#ROS2 packages
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from .ackermann_status import AckermannStatus

#threading package
import threading

#math packages
import math

#keyboard packages
from sshkeyboard import listen_keyboard, stop_listening

#increment definitions:

SPEED_STEPS = 0.01
ANGLE_STEPS = (math.pi/(2.0*100.0))

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')

        #Ackermann object
        self._ackermann_status = AckermannStatus()
        self._ackermann_status.set_angle_step(ANGLE_STEPS)
        self._ackermann_status.set_speed_step(SPEED_STEPS)

        self._publisher = self.create_publisher(AckermannDrive, 'ackermann_input', 10)
        timer_period = 0.1  # seconds
        
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #thread system for keyboard
        self._lock = threading.Lock()
        self._key_buff = []
        self._continue = True;
        self._key_thread = threading.Thread(target=listen_keyboard, 
        kwargs={"on_press":self._press_event,
                "delay_second_char":0.01,
                "delay_other_chars":0.01,})

        self._key_thread.start()


    def _press_event(self, key):
        with self._lock:
            #print(f"'{key}' pressed")
            self._key_buff.append(key)


    def stop(self):
        if self._key_thread is None:
            return

        with self._lock:
            stop_listening()
            self._key_thread.join()
        self._key_thread = None


    def __del__(self):
        self.stop()

    
    def pull_keys(self):
        keys = []
        with self._lock:
            keys = self._key_buff
            self._key_buff = []
        return keys
    

    def _process_key_arrows(self, k):
        if k == "up":
            self._ackermann_status.increase_speed()
        elif k == "down":
            self._ackermann_status.decrease_speed()
        elif k == "right":
            self._ackermann_status.increase_angle()
        elif k == "left":
            self._ackermann_status.decrease_angle()
        else:
            print("fuck: "+k)
    

    def process_events(self):
        
        keys = self.pull_keys()
        for k in keys:
            self._process_key_arrows(k)
            
    def send_ackermann_msg(self):
        speed, angle = self._ackermann_status.pull_status()
        if speed is not None:
            order = AckermannDrive()
            order.speed = speed
            order.steering_angle = angle
            self._publisher.publish(order)
            self.get_logger().info('Publishing Ackermann Msg:' + str(order))

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        self.process_events()
        self.send_ackermann_msg()

def program_exit(keyboard_publisher):
    keyboard_publisher.stop()
    keyboard_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    sys.exit()

    

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
  
    # Create the node
    keyboard_publisher = KeyboardPublisher()
    try:
        # Spin the node so the callback function is called.
        rclpy.spin(keyboard_publisher)
    except KeyboardInterrupt:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        pass
    finally:
        program_exit(keyboard_publisher)
  
if __name__ == '__main__':
  main()