"""
Module to control a Parallax Servo.
This is a template only and needs to be finished in Lab2
"""
from .pwm import Pwm
import time

class Servo:
    def __init__(self, number):
        """Constructor.

        Args:
            number (integer): PWM number where the servo is connected to.
        """
        self._pwm = Pwm(number)
        self._pwm.set_frequency(50)
        self._pwm.set_duty_cycle(7.5)
        self._pwm.enable()

    # def __del__(self):
    #     self._pwm.disable()

    def go_to(self, angle):
        self._pwm.set_duty_cycle(angle / 90. * 3.75 + 7.5)
