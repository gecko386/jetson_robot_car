#!/usr/bin/env python3

import math

"""
This file contains measures and methods for the kinematics of the Dagu Rover 5

"""


#measures in CM
#angels in radians

#distance between wheels

D = 155.0

#Wheel radius, in cm
R = 30

#linear speed in m/s
MAX_LINEAR_SPEED = 1000.0/3600.0

#angular speed in rad/s
MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED/R

ENCODER_RESOLUTION = float(3*2*math.pi)/1000.0


class Rover5(object):

    def __init__(self):
        pass

    def get_global_linear_speed(self, ur, ul):
        return (ur+ul)/2.0

    def get_global_angular_speed(self, ur, ul):
        return (ur-ul)/D

    def get_motor_speeds(self, u, w):
        #return u + D*w/2.0, u - D*w/2.0
        u1 = u + w*2/math.pi
        u2 = u - w*2/math.pi

        if u1 > 1.0:
            d = u1 - 1.0
            u2 = u2 - d/2.0
            u1 = 1.0

        elif u1 < -1.0:
            d = u1 + 1.0
            u2 = u2 - d/2.0
            u1 = -1.0

        if u2 > 1.0:
            d = u2 - 1.0
            u1 = u1 - d/2.0
            u2 = 1.0

        elif u2 < -1.0:
            d = u2 + 1.0
            u1 = u1 - d/2.0
            u2 = -1.0

        return u1, u2

    def normalize_linear_speed(self, u):
        return u/MAX_LINEAR_SPEED

    def normalize_angular_speed(self, w):
        return w/MAX_ANGULAR_SPEED

    def ticks_to_angular_speed(ticks, time):
        angle = ticks*ENCODER_RESOLUTION
        w = angle/time
        return w

    def get_motor_linear_speed(w):
        return w*R