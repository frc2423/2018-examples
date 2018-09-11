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
        self.joystick2 = wpilib.Joystick(1)

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
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""

        # drivetrain teleop
        self.robot_drive.arcadeDrive(self.joystick.getX(), self.joystick.getY())

        # intake teleop
        l_arm_speed = 0
        r_arm_speed = 0

        if self.joystick2.getTrigger():
            if self.joystick2.getY() > 0:
                # intake
                l_arm_speed = -.6
                r_arm_speed = .6
            elif self.joystick2.getY() < 0:
                # outtake
                l_arm_speed = .7
                r_arm_speed = -.7

        self.l_arm.set(l_arm_speed)
        self.r_arm.set(r_arm_speed)

        # elevator teleop
        elevator_speed = -0.3

        if self.joystick2.getRawButton(2):
            # go down
            elevator_speed = 0.25
        elif self.joystick2.getRawButton(3):
            # go up
            self.speed = -.8

        self.elevator_motor.set(elevator_speed)





if __name__ == "__main__":
    wpilib.run(MyRobot)