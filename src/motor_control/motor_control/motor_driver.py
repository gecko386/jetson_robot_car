#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 17 00:30:18 2021

@author: nvidia
"""
# -*- coding: utf-8 -*-

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


# Motor speeds for this library are specified as numbers between -MAX_SPEED and
# MAX_SPEED, inclusive.
# This has a value of 480 for historical reasons/to maintain compatibility with
# older libraries for other Pololu boards (which used WiringPi to set up the
# hardware PWM directly).
_max_speed = 480
MAX_SPEED = _max_speed

_pin_M1FLT = 5
_pin_M2FLT = 6
_pin_M1PWM = 12
_pin_M2PWM = 13
_pin_M1EN = 22
_pin_M2EN = 23
_pin_M1DIR = 24
_pin_M2DIR = 25


class Motor(object):
    MAX_SPEED = _max_speed

    def __init__(self, pwm_pin, dir_pin, en_pin, flt_pin):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.en_pin = en_pin
        self.flt_pin = flt_pin
        
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(flt_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
        GPIO.setup(en_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(dir_pin, GPIO.OUT, initial=GPIO.HIGH) # enable driver by default
        GPIO.setup(pwm_pin, GPIO.OUT, initial=GPIO.HIGH)
        # 20 kHz PWM
        self.pwm = GPIO.PWM(pwm_pin, 100)
        self.pwm.start(0)
        
    def setSpeed(self, speed):
        if speed < 0:
            speed = -speed
            dir_value = GPIO.HIGH
        else:
            dir_value = GPIO.LOW

        if speed > 1.0:
            speed = 1.0

        GPIO.output(self.dir_pin, dir_value)
        print(speed * 100.0)
        self.pwm.ChangeDutyCycle(int(speed * 100.0))

    def enable(self):
        GPIO.output(self.en_pin, GPIO.HIGH)

    def disable(self):
        GPIO.output(self.en_pin, GPIO.LOW)

    def getFault(self):
        return not GPIO.input(self.flt_pin)
    
    def __del__(self):
        self.pwm.stop()

class Motors(object):
    MAX_SPEED = _max_speed

    def __init__(self):
        self.motor1 = Motor(_pin_M1PWM, _pin_M1DIR, _pin_M1EN, _pin_M1FLT)
        self.motor2 = Motor(_pin_M2PWM, _pin_M2DIR, _pin_M2EN, _pin_M2FLT)

    def __del__(self):
        del self.motor1
        del self.motor2        
        GPIO.cleanup()

    def setSpeeds(self, m1_speed, m2_speed):
        self.motor1.setSpeed(m1_speed)
        self.motor2.setSpeed(m2_speed)

    def enable(self):
        self.motor1.enable()
        self.motor2.enable()

    def disable(self):
        self.motor1.disable()
        self.motor2.disable()

    def getFaults(self):
        return self.motor1.getFault() or self.motor2.getFault()

    def forceStop(self):
        # reinitialize the pigpio interface in case we interrupted another command
        # (so this method works reliably when called from an exception handler)
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        self.motor1 = Motor(_pin_M1PWM, _pin_M1DIR, _pin_M1EN, _pin_M1FLT)
        self.motor2 = Motor(_pin_M2PWM, _pin_M2DIR, _pin_M2EN, _pin_M2FLT)
        self.setSpeeds(0, 0)


