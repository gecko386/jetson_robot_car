#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from .quadrature_encoder import Encoder

class WheelKinePublisher(Node):
    def __init__(self):
        super().__init__('wheelkine_publisher')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('left_pin', None),
                ('right_pin', None),
            ])