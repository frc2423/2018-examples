#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctres


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

        self.joystick = wpilib.Joystick(1)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        if self.joystick.getRawButton(1):
            # Drive forwards
            self.robot_drive.arcadeDrive(0, .5)
        elif self.joystick.getRawButton(2):
            # Drive backwards
            self.robot_drive.arcadeDrive(0, -.5)
        elif self.joystick.getRawButton(3):
            # Turn left
            self.robot_drive.arcadeDrive(-.5, 0)
        elif self.joystick.getRawButton(4):
            # Turn right
            self.robot_drive.arcadeDrive(.5, 0)
        else:
            self.robot_drive.arcadeDrive(0, 0)


if __name__ == "__main__":
    wpilib.run(MyRobot)