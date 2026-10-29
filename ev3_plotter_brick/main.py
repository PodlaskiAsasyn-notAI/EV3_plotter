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
        self.second_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)

        # configure strict motor tolerances
        self.first_motor.control.target_tolerances(5, 1)
        self.second_motor.control.target_tolerances(5, 1)


        #put in measured axle to axle distance (in mm)
        self.arm_a = 200
        self.arm_b = 200

        # put in corner coords and draw region dimentions
        self.corner = (100, -50)
        self.box_size = (100, 100)

        # set the phisical limits for servo motors
        self.limit_alpha = (0, 180)
        self.limit_beta = (0, 180)

        # self.first_motor.reset_angle(0)
        # self.second_motor.reset_angle(0)

    def calc_angles(self, x, y):
        # validation
        r = math.sqrt(x**2 + y**2)
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
        return (alpha, beta)

    def translate_coords(self, x, y):
        # translacja koordynatow na uzywanego boxa
        x += self.corner[0]
        y += self.corner[1]
        return (x, y)

    def goto(self, x, y, speed=30):
        x, y = self.translate_coords(x, y)
        angles = self.calc_angles(x, y)
        if not angles:
            return None
        alpha, beta = angles

        self.second_motor.run_target(speed, beta, wait=False)
        self.first_motor.track_targer(speed, alpha, wait=True)

        return None

    def draw_line(self, x0, y0, x1, y1, speed=30):
        """
        Draws a straight line between two points. Steers the engines via calculating optimal omegas.
        """
        # translacja koordynatow na uzywanego boxa
        x0, y0 = self.translate_coords(x0, y0)
        x1, y1 = self.translate_coords(x1, y1)

        # obliczanie i walidacja potrzebnych kątów
        angles_0 = self.calc_angles(x0, y0)
        angles_1 = self.calc_angles(x1, y1)
        if not angles_0 or not angles_1:
            return None
        alpha_0, beta_0 = angles_0
        alpha_1, beta_1 = angles_1

        # korekta pozycji
        if abs(self.first_motor.angle() - alpha_0) < 10 or abs(self.second_motor.angle() - beta_0) < 10:
            self.goto(x0, y0)

        total_distance = math.sqrt(abs(x0 - x1)**2 + abs(y0 - y1))
        total_distance = round(total_distance, 2)

        total_time = total_distance / speed

        delta_alpha = alpha_1 - alpha_0
        omega_alpha = delta_alpha / total_time

        delta_beta = beta_1 - beta_1
        omega_beta = delta_beta / total_time

        self.first_motor.run_time(omega_alpha, total_time, then=Stop.STOP, wait=False)
        self.second_motor.run_time(omega_beta, total_time, then=Stop.STOP, wait=True)
        return 1

    def go_home(self):
        """
        Homes the arm to default location (alpha = 0, beta = 0)
        """
        x, y = self.translate_coords(0, 0)
        x += self.box_size[0]
        y += (self.box_size[1] / 2)
        self.goto(x, y)

    def check_poossible_range(self, ran=None, step=10, save=False, path="possible_locations.txt"):
        """
        Goes through all the points, range from absolute point (0, 0) step apart from each other.
        If range is not specified, then it's automatically calculated.
        If save = True, output is saved to a file specified by path.
        """
        output = []
        if ran is None:
            ran = self.arm_a + self.arm_b

        for i in range(-ran, ran + 1, step):
            for j in range(-ran, ran + 1, step):
                angles = self.calc_angles(i, j)
                if angles:
                    output.append((i, j))
        if save:
            with open(path, "w", encoding="utf-8") as file:
                output_str = [str(line) + '/n' for line in output]
                file.writelines(output_str)
        return output

    def draw_box(self):
        """
        Draws a maximum possible rectangle
        """
        # bottom right
        x0, y0 = self.corner
        # bottom left
        x1 = self.corner[0] + self.box_size[0]
        y1 = self.corner[1]
        # top left
        x2 = self.corner[0] + self.box_size[0]
        y2 = self.corner[1] + self.box_size[1]
        # top right
        x3 = self.corner[0]
        y3 = self.corner[1] + self.box_size[1]

        self.draw_line(x0, y0, x1, y1)
        self.draw_line(x1, y1, x2, y2)
        self.draw_line(x2, y2, x3, y3)
        self.draw_line(x3, y3, x0, y0)




def main():
    arm = Arm()


if __name__ == "__main__":
    main()
