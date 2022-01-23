#!/usr/bin/env python3
import math

class AckermannStatus(object):

    
    def __init__(self):

        self._last_pulled_angle = 0.0
        self._last_pulled_speed = 0.0

        self._current_angle = 0.0
        self._current_speed = 0.0

        #angle value under this will be converted to -Pi/2
        self._angle_min_clamp_value = 0.1

        #angle value over this will be converted to Pi/2
        self._angle_max_clamp_value = math.pi/2.0

        #speed value under this will be converted to 
        self._speed_min_clamp_value = 0.1

        #speed value over this will be converted to one
        self._speed_max_clamp_value = 1.0

        #step increment for angle
        self._angle_step = (math.pi/(2.0*100.0))

        #step increment for speed
        self._speed_step = 0.01

    def _changes_since_last_pull(self):
        return (self._last_pulled_angle != self._current_angle 
            or self._last_pulled_speed != self._current_speed)


    def _clamp_to_zero(self, value, clamp_value):
        if abs(value) < clamp_value:
            return 0.0
        else:
            return value

    def _clamp_to_one(self, value, clamp_value):
        if value > clamp_value:
            return 1.0
        elif value < -clamp_value:
            return -1
        else:
            return value

    def _clamp_to_pi_half(self, value, clamp_value):
        if value > clamp_value:
            return math.pi/2.0
        elif value < -clamp_value:
            return -math.pi/2.0
        else:
            return value
 
    def _clamp_angle_value(self, value):
        return self._clamp_to_pi_half(
            self._clamp_to_zero(value, self._angle_min_clamp_value), 
            self._angle_max_clamp_value)

    def _clamp_speed_value(self, value):
        return self._clamp_to_one(
            self._clamp_to_zero(value, self._speed_min_clamp_value), 
            self._speed_max_clamp_value)

    def set_speed_min_value(self, value):
        if value >= 0.0:
            self._speed_min_clamp_value = value

    def set_speed_max_value(self, value):
        if value <= 1.0:
            self._speed_max_clamp_value = value

    def set_angle_min_value(self, value):
        if value >= 0.0:
            self._angle_min_clamp_value = value

    def set_angle_max_value(self, value):
        if value <= 1.0:
            self._angle_max_clamp_value = value

    def set_angle_step(self, value):
        self._angle_stetp = value

    def set_speed_step(self, value):
        self._speed_step = value

    def pull_status(self):
        if (self._changes_since_last_pull()):
            self._last_pulled_angle = self._current_angle
            self._last_pulled_speed = self._current_speed

            return self._current_speed, self._current_angle
        else:
            return None, None

    def get_last_pulled_speed(self):
        return self._last_pulled_speed

    def get_last_pulled_angle(self):
        return self._last_pulled_angle

    def set_speed(self, speed):
        self._current_speed = self._clamp_speed_value(speed)

    def set_angle(self, angle):
        self._current_angle = self._clamp_angle_value(angle)

    def increase_speed(self):
        if self._current_speed < 1.0:
            if self._current_speed == 0.0:
                self._current_speed = self._speed_min_clamp_value
            self._current_speed = self._clamp_speed_value(self._current_speed + self._speed_step)

    def decrease_speed(self):
        if self._current_speed > -1.0:
            if self._current_speed == 1.0:
                self._current_speed = self.speed_max_clamp_value
            elif self._current_speed == 0.0:
                self._current_speed = -self._speed_min_clamp_value
            self._current_speed = self._clamp_speed_value(self._current_speed - self._speed_step)

    def increase_angle(self):
        if self._current_angle < math.pi/2.0:
            if self._current_angle == 0.0:
                self._current_angle = self._angle_min_clamp_value
            self._current_angle = self._clamp_angle_value(self._current_angle + self._angle_step)

    def decrease_angle(self):
        if self._current_angle > -math.pi/2.0:
            if self._current_angle == math.pi/2.0:
                self._current_angle = self._angle_max_clamp_value
            elif self._current_angle == 0.0:
                self._current_angle = -self._angle_min_clamp_value
            self._current_angle = self._clamp_angle_value(self._current_angle - self._angle_step)

    

    