#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import math

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.


# Write your program here.
class Arm():
    def __innit__(self):
        ev3 = EV3Brick()

        # Initialize the motors.
        self.first_motor = Motor(Port.B, Direction.CLOCKWISE)
        self.second_motor = Motor(Port.C, Direction.CLOCKWISE)

        #put in measured axle to axle distance (in mm)
        self.arm_a = 200
        self.arm_b = 200


        # set the phisical limits for servo motors
        self.limit_alpha = (0, 180)
        self.limit_beta = (0, 180)

        self.first_motor.reset_angle(0)
        self.second_motor.reset_angle(0)

    def move_first_motor(self, deg, dir='+'):
        target = 0
        if dir == '+':
            target = self.first_motor.angle() + deg
        if dir == '-':
            target = self.first_motor.angle() - deg
        self.first_motor.track_target(target)

    def move_second_motor(self, deg, dir='+'):
        target = 0
        if dir == '+':
            target = self.second_motor.angle() + deg
        if dir == '-':
            target = self.second_motor.angle() - deg
        self.second_motor.track_target(target)

    def goto(self, x, y):
        # validation
        r = math.hypot(x, y)
        if r > (self.arm_a + self.arm_b) or r < abs(self.arm_a - self.arm_b):
            return None

        cos_angle = (x**2 + y**2 - self.arm_a**2 - self.arm_b**2) / (2*self.arm_a*self.arm_b)
        cos_angle = max(-1, min(1, cos_angle))

        beta = -math.acos(cos_angle)
        alpha = math.atan2(y, x) + math.atan2((self.arm_b*math.sin(beta)), (self.arm_a + self.arm_b * math.cos(beta)))

        beta = math.degrees(beta)
        alpha = math.degrees(alpha)

        if self.limit_beta[0] > beta or self.limit_beta[1] < beta:
            return None
        if self.limit_alpha[0] > alpha or self.limit_alpha[1] < alpha:
            return None

        self.second_motor.track_target(beta)
        self.first_motor.track_targer(alpha)

        return None


def main():
    arm = Arm()
    arm.move_first_motor(90, '+')
    arm.move_first_motor(90, '-')
    arm.move_second_motor(90, '+')
    arm.move_second_motor(90, '-')


if __name__ == "__main__":
    main()
