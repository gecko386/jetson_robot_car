#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self._publisher = self.create_publisher(CompressedImage, 'video_frames', 10)
        timer_period = 0.01  # seconds
      
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
            
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)
            
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()
        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self._publisher.publish(self.br.cv2_to_compressed_imgmsg(frame))
            #self.br.cv2_to_compressed_imgmsg
    
        # Display the message on the console
        self.get_logger().info('Publishing video frame')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
  
    # Create the node
    camera_publisher = CameraPublisher()
  
    # Spin the node so the callback function is called.
    rclpy.spin(camera_publisher)
  
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()
  
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()