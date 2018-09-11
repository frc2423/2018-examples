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

        self.gyro = navx.AHRS.create_spi()


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
        desired_angle = math.atan2(y, x)

        angle_distance = desired_angle - current_angle

        if angle_distance > 180:
            angle_distance -= 360
        elif angle_distance < 180:
            angle_distance += 360

        # bang-bang controller
        '''
        if angle_distance > 0:
            turn_rate = .5
        else:
            turn_rate = -.5
        '''

        # proportional controller
        turn_rate = (angle_distance / 180) * 1

        self.robot_drive.arcadeDrive(turn_rate, 0)





if __name__ == "__main__":
    wpilib.run(MyRobot)