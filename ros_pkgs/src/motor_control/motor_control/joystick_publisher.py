#!/usr/bin/env python3

#ROS2 packages
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from .ackermann_status import AckermannStatus

#pygame joystick packages
import os
import sys
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame
#from pygame import joystick, event
#from pygame import QUIT, JOYAXISMOTION, JOYBALLMOTION, JOYHATMOTION, JOYBUTTONUP, JOYBUTTONDOWN

#math packages
import math

#axis definition
X_AXIS = 0
Y_AXIS_POS = 2
Y_AXIS_NEG = 5

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        
        #Ackermann object
        self._ackermann_status = AckermannStatus()
        self._ackermann_status.set_angle_min_value(0.4)
        
        self._publisher = self.create_publisher(AckermannDrive, 'ackermann_input', 10)
        timer_period = 0.1  # seconds
            
        # Create the joystick 
        pygame.init()
        pygame.display.set_mode((800, 600))
        pygame.display.set_caption("My Game")
        clock = pygame.time.Clock()
        
        pygame.joystick.init()
        pygame.key.set_repeat(10,10)
        joystick_count = pygame.joystick.get_count()

        joy_found = False
        for i in range(joystick_count):
            joy = pygame.joystick.Joystick(i)
            joy.init()
            if joy.get_init():
                joy_found = True
                break
        
        if not joy_found:
            raise Exception("No joystick found!!!")

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def update_last_Axis(self, e):
        if e.axis == X_AXIS:
            self._ackermann_status.set_angle(e.value * math.pi/2.0)

        elif e.axis == Y_AXIS_POS:
            if self._ackermann_status.get_last_pulled_speed() < 0.0:
                return
            self._ackermann_status.set_speed((e.value+1.0)/2.0)

        elif e.axis == Y_AXIS_NEG:
            if self._ackermann_status.get_last_pulled_speed() > 0.0:
                return
            self._ackermann_status.set_speed(-(e.value+1.0)/2.0)


    def send_ackermann_msg(self):

        speed, angle = self._ackermann_status.pull_status()
        if speed is not None:
            order = AckermannDrive()
            order.speed = speed
            order.steering_angle = angle
            self._publisher.publish(order)
            self.get_logger().info('Publishing Ackermann Msg:' + str(order))
    

    def process_events(self):

        #time.sleep(1.0 / 20.0)
        for e in pygame.event.get():

            #print(e.type)

            if e.type == pygame.JOYAXISMOTION:
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
        pygame.display.flip()


    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        self.process_events()        
        self.send_ackermann_msg()

def program_exit(joystick_publisher):
    joystick_publisher.destroy_node()
    del joystick_publisher

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    sys.exit()

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)


    # Create the node
    joystick_publisher = JoystickPublisher()
    try:
        # Spin the node so the callback function is called.
        rclpy.spin(joystick_publisher)
    except KeyboardInterrupt:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        pass
    finally:
        program_exit(joystick_publisher)
  
if __name__ == '__main__':
  main()