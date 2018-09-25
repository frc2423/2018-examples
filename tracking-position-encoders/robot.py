#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
import math


class MyRobot(wpilib.TimedRobot):

    # These measurements are in inches
    WHEEL_RADIUS = 3
    DRIVE_BASE = 28

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

        self.prev_left_angular_pos = 0
        self.prev_right_angular_pos = 0

        self.angle = 0

        self.x_pos = 0
        self.y_pos = 0



    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.robot_drive.arcadeDrive(0, 0)

    def teleopInit(self):
        self.fl_motor.setQuadraturePosition(0, 0)
        self.br_motor.setQuadraturePosition(0, 0)
        self.x_pos = 0
        self.y_pos = 0
        self.angle = 0
        self.prev_left_angular_pos = self.get_left_angular_pos()
        self.prev_right_angular_pos = self.get_right_angular_pos()


    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        dt = self.getPeriod()

        left_angular_pos = self.get_left_angular_pos()
        right_angular_pos = self.get_right_angular_pos()

        # calculate angular velocities of the wheels
        left_angular_vel = (left_angular_pos - self.prev_left_angular_pos) / dt
        right_angular_vel = (right_angular_pos - self.prev_right_angular_pos) / dt

        # calculate the velocity of the robot
        vel = (left_angular_vel + right_angular_vel) * MyRobot.WHEEL_RADIUS / 2

        # calculate angular velocity and use it to calculate the new heading of the robot
        angular_vel = (right_angular_vel - left_angular_vel) * MyRobot.WHEEL_RADIUS / MyRobot.DRIVE_BASE
        self.angle += angular_vel * dt

        # Update the robot's position
        self.x_pos += vel * math.sin(self.angle) * dt
        self.y_pos += vel * math.cos(self.angle) * dt

        print('position:', self.x_pos, ', ', self.y_pos)
        print('angle:', self.angle)

        # save these values so we can take the difference between them. We want to measure
        # the difference in order to calculate the angular velocity.
        self.prev_left_angular_pos = left_angular_pos
        self.prev_right_angular_pos = right_angular_pos


    def get_left_angular_pos(self):
      encoder_value = self.fl_motor.getQuadraturePosition()
      ticks_per_turn = -1000
      return 2 * math.pi * encoder_value / ticks_per_turn


    def get_right_angular_pos(self):
        encoder_value = self.br_motor.getQuadraturePosition()
        ticks_per_turn = 1440
        return 2 * math.pi * encoder_value / ticks_per_turn

if __name__ == "__main__":
    wpilib.run(MyRobot)