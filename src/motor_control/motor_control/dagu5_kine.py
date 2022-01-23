#!/usr/bin/env python3

"""
This file contains measures and methods for the kinematics of the Dagu Rover 5

"""



#measures in CM

#distance between wheels
D = 155.0

#Wheel radius, in cm
R = 30

#linear speed in m/s
MAX_LINEAR_SPEED = 0.25

#angular speed in rad/s
MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED/R


class Rover5(object):

    def __init__(self):
        pass

    def get_global_linear_speed(self, ur, ul):
        return (ur+ul)/2.0

    def get_global_angular_speed(self, ur, ul):
        return (ur-ul)/D

    def get_motor_speeds(self, u, w):
        return u + D*w/2.0, u - D*w/2.0 

    def normalize_linear_speed(self, u):
        return u/MAX_LINEAR_SPEED

    def normalize_angular_speed(self, w):
        return w/MAX_ANGULAR_SPEED

