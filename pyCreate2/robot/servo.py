"""
Module to control a Parallax Servo.
This is a template only and needs to be finished in Lab2
"""

from .pwm import Pwm


class Servo:
    def __init__(self, number):
        self.p = Pwm(number)
        self.p.set_frequency(50)
        """Constructor.
        Args:
        number (integer): PWM number where the servo is connected to.
        """

    def go_to(self, angle):
        self.p.enable()
        if angle >= 0:
            self.p.set_duty_cycle((angle/90 * 0.75/20 + 1.5/20)*100)
        if angle < 0:
            self.p.set_duty_cycle((1.5/20 - (-angle/90 * 0.75/20))*100)


