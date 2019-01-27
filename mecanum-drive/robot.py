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
        self.br_motor = ctre.wpi_talonsrx.WPI_TalonSRX(40)
        self.bl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(50)
        self.fl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(30)
        self.fr_motor = ctre.wpi_talonsrx.WPI_TalonSRX(10)

        self.fr_motor.configMotionAcceleration()
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
        x = self.joystick.getRawAxis(0)
        y = self.joystick.getRawAxis(1)
        turn = self.joystick.getRawAxis(4)

        self.robot_drive.mecanumDrive_Cartesian(x, y, turn, 0)


if __name__ == "__main__":
    wpilib.run(MyRobot)