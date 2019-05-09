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

    min_tx = ntproperty('/limelight/min_tx', 0, persistent = True)
    max_tx = ntproperty('/limelight/max_tx', 0, persistent = True)
    min_ta = ntproperty('/limelight/min_ta', 1, persistent = True)
    max_ta = ntproperty('/limelight/max_ta', 15, persistent = True)

    speed_multiplier = ntproperty('/limelight/speed_multiplier', 1, persistent = True)
    turn_multiplier = ntproperty('/limelight/turn_multiplier', 1, persistent = True)
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

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.robot_drive.arcadeDrive(0, 0)

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""

        if self.ta > self.min_ta and self.ta < self.max_ta and self.tv:
          #changing target area to a value between -.5 to .5 in order to plug into arcade drive
          max_speed = self.max_ta - self.min_ta
          divide_speed = (self.max_ta - self.min_ta)
          speed = ((max_speed - self.ta) / divide_speed) - .5

          print('speed: ', speed)

          self.robot_drive.arcadeDrive(self.tx * self.turn_multiplier / 20, -speed * self.speed_multiplier)

        else:
          print('target not detected')
          self.robot_drive.arcadeDrive(self.joystick.getX(), self.joystick.getY())



if __name__ == "__main__":
    wpilib.run(MyRobot)