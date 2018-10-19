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

        self.l_arm = wpilib.Spark(0)
        self.r_arm = wpilib.Spark(1)

        self.elevator_motor = ctre.wpi_talonsrx.WPI_TalonSRX(4)
        self.elevator_follower = ctre.wpi_talonsrx.WPI_TalonSRX(10)
        self.elevator_follower.follow(self.elevator_motor)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.robot_drive.arcadeDrive(0, 0)

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        print(self.joystick.getPOVCount())
        speed = 0
        if self.joystick.getRawButton(5) and self.joystick.getRawButton(6):
            speed = -1
        elif self.joystick.getRawButton(5):
            speed = -.5
        elif self.joystick.getRawButton(6):
            speed = .5



        self.robot_drive.arcadeDrive(self.joystick.getRawAxis(0), speed)


        # elevator
        elevator_speed = -0.2

        if self.joystick.getPOV():
            # go down
            elevator_speed = 0.35
        elif self.joystick.getRawButton(2):
            # go up
            elevator_speed = -.8

        self.elevator_motor.set(elevator_speed)

        # arms
        l_arm_speed = 0
        r_arm_speed = 0

        if self.joystick.getRawButton(2):
            # intake
            l_arm_speed = -.6
            r_arm_speed = .6
        elif self.joystick.getRawButton(4):
            # outtake
            l_arm_speed = .7
            r_arm_speed = -.7

        self.l_arm.set(l_arm_speed)
        self.r_arm.set(r_arm_speed)





if __name__ == "__main__":
    wpilib.run(MyRobot)