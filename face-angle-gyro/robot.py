#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
from robotpy_ext.common_drivers import navx
import math


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.br_motor = ctre.wpi_talonsrx.WPI_TalonSRX(1)
        self.bl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(2)
        self.fl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(5)
        self.fr_motor = ctre.wpi_talonsrx.WPI_TalonSRX(7)
        self.fr_motor.setInverted(True)
        self.br_motor.setInverted(True)

        self.robot_drive = wpilib.RobotDrive(self.fl_motor, self.bl_motor, self.fr_motor, self.br_motor)

        self.joystick = wpilib.Joystick(0)
        self.JOYSTICK_DEADZONE_THRESHOLD = .5

        #self.gyro = navx.AHRS.create_spi()
        self.gyro = wpilib.ADXRS450_Gyro()



    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        current_angle = self.gyro.getAngle() % 360

        x = self.joystick.getX()
        y = self.joystick.getY()
        joystick_magnitude = math.sqrt(x**2 + y**2)
        desired_angle = math.atan2(x, -y) * 180 / math.pi
        angle_distance = desired_angle - current_angle

        if angle_distance > 180:
            angle_distance -= 360
        elif angle_distance < -180:
            angle_distance += 360

        # bang-bang controller
        if angle_distance > 0:
            turn_rate = .5
        else:
            turn_rate = -.5


        print('angle_distance:', angle_distance)

        # proportional controller
        #turn_rate = (angle_distance / 180) * 1

        if joystick_magnitude < self.JOYSTICK_DEADZONE_THRESHOLD:
            self.robot_drive.arcadeDrive(0, 0)
        else:
            self.robot_drive.arcadeDrive(turn_rate, 0)





if __name__ == "__main__":
    wpilib.run(MyRobot)