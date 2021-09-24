#!/usr/bin/env python3

#ROS2 packages
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive

#pygame joystick packages
import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame
from pygame import joystick, event
from pygame import QUIT, JOYAXISMOTION, JOYBALLMOTION, JOYHATMOTION, JOYBUTTONUP, JOYBUTTONDOWN

#math packages
import math

#axis definition
X_AXIS = 0
Y_AXIS_POS = 2
Y_AXIS_NEG = 5

MIN_AXIS_VAL = 0.1
MAX_AXIS_VAL = 1.0

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        
        #Ackermann orders
        self._last_angle = 0.0
        self._last_speed = 0.0
        self._need_update_ackermann = True
        
        
        self._publisher = self.create_publisher(AckermannDrive, 'ackermann_input', 10)
        timer_period = 0.1  # seconds
      
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
            
        # Create the joystick 
        pygame.init()
        pygame.display.init()
        pygame.display.set_mode((1, 1))
        joystick.init()
        pygame.key.set_repeat(10,10)

        for i in range(joystick.get_count()):
            joy = joystick.Joystick(i)
            joy.init()
            if joy.get_init():
                break

    def clamp_to_zero(self, value, limit = MIN_AXIS_VAL):
        if abs(value) < limit:
            return 0.0
        else:
            return value

    def clamp_to_one(self, value, limit = MAX_AXIS_VAL):
        if value > limit:
            return limit
        elif value < -limit:
            return -limit
        else:
            return value
 
    def clamp_value(self, value):
        return self.clamp_to_one(self.clamp_to_zero(value))

    def update_last_Axis(self, e):
        if e.axis == X_AXIS:
            angle = self.clamp_to_zero(e.value * math.pi/2.0, 0.4)
            if angle == self._last_angle:
                return

            self._last_angle = angle
            self._need_update_ackermann = True

        elif e.axis == Y_AXIS_POS:
            if self._last_speed < 0.0:
                return

            speed = self.clamp_value((e.value+1.0)/2.0)
            if speed == self._last_speed:
                return

            self._last_speed = speed
            self._need_update_ackermann = True

        elif e.axis == Y_AXIS_NEG:
            if self._last_speed > 0.0:
                return

            speed = -self.clamp_value((e.value+1.0)/2.0)
            if speed == self._last_speed:
                return

            self._last_speed = speed
            self._need_update_ackermann = True

    def send_ackermann_msg(self):
        if self._need_update_ackermann:
            order = AckermannDrive()
            order.steering_angle = self._last_angle
            order.speed = self._last_speed
            self._publisher.publish(order)
            self.get_logger().info('Publishing Ackermann Msg:' + str(order))
            self._need_update_ackermann = False
    


    def process_events(self):

        #time.sleep(1.0 / 20.0)
        for e in pygame.event.get():


            if e.type == JOYAXISMOTION:
                #print("axis event: "+"axis: " + str(e.axis) + " value: "+str(e.value))
                self.update_last_Axis(e)
            
                
            """
            elif e.type == JOYHATMOTION:
                print("hat event: " + "hat: "+str(e.hat) + " hat X: "+str(e.value[0]) + " hat Y: "+str(e.value[0]))
                
            elif e.type == JOYBUTTONUP:
                print("joy up event: " + "joy button: " + str(e.button))
                
            elif e.type == JOYBUTTONDOWN:
                print("joy down event: " + "joy button: " + str(e.button))

            elif e.type == KEYUP:
                print("key up event: " + "key up: " + str(e.key))
                
            elif e.type == KEYDOWN:
                print("key down event: " + "key down: " + str(e.key))
            """
    


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
    joystick_publisher = JoystickPublisher()
  
    # Spin the node so the callback function is called.
    rclpy.spin(joystick_publisher)
  
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick_publisher.destroy_node()
  
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()