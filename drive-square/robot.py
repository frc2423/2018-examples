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
        self.state = 'forward'
        self.timer = wpilib.Timer()
        self.timer.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        if self.state == 'forward':
            # perform action
            self.robot_drive.arcadeDrive(0, -.5)

            # transition to the next state after 3 seconds
            if self.timer.get() > 2:
                self.state = 'turn'
                self.timer.reset()

        elif self.state == 'turn':
            # perform action
            self.robot_drive.arcadeDrive(.5, 0)

            # Transition to the next state after robot has turned 90 degrees using dead reckoning
            if self.timer.get() > 1.4:
                self.state = 'forward'
                self.timer.reset()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.robot_drive.arcadeDrive(self.joystick.getX(), self.joystick.getY())


if __name__ == "__main__":
    wpilib.run(MyRobot)