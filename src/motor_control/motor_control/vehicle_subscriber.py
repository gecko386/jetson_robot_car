#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from .motor_driver import Motors
import math

class VehicleSubscriber(Node):
    """
    Create an VehicleSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('vehicle_subscriber')
        
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self._subscriber = self.create_subscription(AckermannDrive, 'ackermann_input', self.process_events, 10)

        self._motors = Motors()
        self._motors.enable()

    def ackermann_to_motors(self, ackermann_order):
        max_speed = self._motors.MAX_SPEED

        angle = ackermann_order.steering_angle
        speed = ackermann_order.speed

        s1 = speed
        s2 = speed

        if angle>0:
          s2 *= math.cos(angle)
        elif angle<0:
          s1 *= math.cos(-angle)


        print("Speeds: "+str(s1)+", "+str(s2))
        return s1*max_speed, s2*max_speed

    def process_events(self, data):
        """
        Callback function.
        """
        m1, m2 = self.ackermann_to_motors(data)
        self._motors.setSpeeds(m1, m2)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  vehicle_subscriber = VehicleSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(vehicle_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  vehicle_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()