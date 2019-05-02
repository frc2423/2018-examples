#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
from networktables import NetworkTables
from networktables.util import ntproperty


class MyRobot(wpilib.TimedRobot):

    slowdown_multiplier = ntproperty('/babby_bot/slowdown_multiplier', 1, persistent = True)
    turning_slowdown_multiplier = ntproperty('/babby_bot/slowdown_multiplier', 1, persistent = True)

    tv = ntproperty('/limelight/tv', 0)
    ta = ntproperty('/limelight/ta', 0)
    tx = ntproperty('/limelight/tx', 0)

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.br_motor = ctre.wpi_talonsrx.WPI_TalonSRX(5)
        self.bl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(4)
        self.fr_motor = ctre.wpi_talonsrx.WPI_TalonSRX(7)
        self.fl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(3)
        self.fr_motor.setInverted(True)
        self.br_motor.setInverted(True)

        self.robot_drive = wpilib.RobotDrive(self.fl_motor, self.bl_motor, self.fr_motor, self.br_motor)

        self.joystick = wpilib.Joystick(0)

        self.min_ta = 3
        self.max_ta = 11

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.robot_drive.arcadeDrive(0, 0)

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""

        if self.ta > self.min_ta and self.ta < self.max_ta and self.tv:
          max_speed = self.max_ta - self.min_ta
          divide_speed = (self.max_ta - self.min_ta) * 2
          speed = (max_speed - self.ta) / divide_speed

          print('speed: ', speed)

          self.robot_drive.arcadeDrive(speed, 0)

        else:
          print('target not detected')
          self.robot_drive.arcadeDrive(0, 0)


if __name__ == "__main__":
    wpilib.run(MyRobot)