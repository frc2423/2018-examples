#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre


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

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        turn_rate = 0
        speed = 0

        if self.joystick.getRawButton(3):
            # Drive forwards
            speed = -.5
        elif self.joystick.getRawButton(2):
            # Drive backwards
            speed = .5
        elif self.joystick.getRawButton(4):
            # Turn left
            turn_rate = -.5
        elif self.joystick.getRawButton(5):
            # Turn right
            turn_rate = .5

        self.robot_drive.arcadeDrive(turn_rate, speed)


if __name__ == "__main__":
    wpilib.run(MyRobot)