#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Range

try:
    import RPi.GPIO as GPIO

    output_pins = {
        'JETSON_XAVIER': 18,
        'JETSON_NANO': 33,
        'JETSON_NX': 33,
        'CLARA_AGX_XAVIER': 18,
        'JETSON_TX2_NX': 32,
    }
    output_pin = output_pins.get(GPIO.model, None)
    if output_pin is None:
        raise Exception('PWM not supported on this board')
    print("using real GPIO")
except:
    print("using dummy GPIO")
    from common import gpio_dummy as GPIO



class UltrasoundPublisher(Node):
    def __init__(self):
        super().__init__('ultrasound_publisher')

        self.declare_parameter('ultrasound_id', None, ParameterDescriptor(description='string to identify the ultrasound sensor, this will be the postfix of topics: us_'))
        self.declare_parameter('trigger_pin', None, ParameterDescriptor(description='BCM pin to send ultrasound trigger'))
        self.declare_parameter('echo_pin', None, ParameterDescriptor(description='BCM pin to wait ultrasound response'))


        #configure pins
        GPIO.setup(self.get_parameter('trigger_pin').get_parameter_value().integer_value, GPIO.OUT)
        GPIO.setup(self.get_parameter('echo_pin').get_parameter_value().integer_value, GPIO.IN)

        self._publisher = self.create_publisher(Range, 'us_'+self.get_parameter('ultrasound_id').get_parameter_value().string_value, 10)
        timer_period = 0.05  # seconds
      
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.05 seconds.
        """
        # set Trigger to HIGH
        GPIO.output(self.get_parameter('trigger_pin').get_parameter_value().integer_value, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.get_parameter('trigger_pin').get_parameter_value().integer_value, False)

        start_time = time.time()
        stop_time = time.time()

        # save StartTime
        while GPIO.input(self.get_parameter('echo_pin').get_parameter_value().integer_value) == 0:
            start_time = time.time()

        # save time of arrival
        while GPIO.input(self.get_parameter('echo_pin').get_parameter_value().integer_value) == 1:
            stop_time = time.time()

        # time difference between start and arrival
        time_elapsed = stop_time - start_time
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (time_elapsed * 34300) / 2

        range = Range()
        range.min_range = 1.9
        range.max_range = 450
        range.radiation_type=Range.ULTRASOUND
        range.range = distance_sensors

        #timestamp
        range.header.stamp = self.get_clock().now().to_msg()

        self._publisher.publish(range)

        self.get_logger().info('Publishing ultrasound_'+self.get_parameter('ultrasound_id').get_parameter_value().string_value+' with value: '+str(distance))


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
  
    # Create the node
    ultrasound_publisher = UltrasoundPublisher()
  
    # Spin the node so the callback function is called.
    rclpy.spin(ultrasound_publisher)
  
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ultrasound_publisher.destroy_node()
  
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()