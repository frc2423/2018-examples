#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
from networktables import NetworkTables
from networktables.util import ntproperty


class MyRobot(wpilib.TimedRobot):

    up_position = ntproperty('/servo/target_x', 1)
    down_position = ntproperty('/servo/target_y', 0)

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

        self.dr_doom_disc = wpilib.Servo(8)

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
        self.robot_drive.arcadeDrive(self.joystick.getX(), self.joystick.getY())

        if self.joystick.getRawButton(5):
            self.dr_doom_disc.set(self.up_position)

        if self.joystick.getRawButton(6):
            self.dr_doom_disc.set(self.down_position)

if __name__ == "__main__":
    wpilib.run(MyRobot)