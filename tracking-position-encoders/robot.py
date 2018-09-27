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

    # In degrees
    THRESHOLD_ANGLE = 5

    # In inches
    THRESHOLD_DISTANCE = 12

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
        self.reset_odometry()


    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.calculate_odometry()

        if self.joystick.getRawButton(3):
            self.reset_odometry()

        if self.joystick.getTrigger():
            self.drive_to(4, 4)
        else:
            self.robot_drive.arcadeDrive(self.joystick.getX(), self.joystick.getY())

        print('position: %1.2f, %1.2f' % self.get_position())
        print('angle: %1.0f' % self.get_angle())

    def drive_to(self, desired_x, desired_y):

        # change from feet to inches
        desired_x *= 12
        desired_y *= 12

        desired_angle = math.atan2(desired_x - self.x_pos, desired_y - self.y_pos) * 180 / math.pi
        distance = math.sqrt((desired_x - self.x_pos) ** 2 + (desired_y - self.y_pos) ** 2)

        angle_distance = desired_angle - self.get_angle()
        if angle_distance > 180:
            angle_distance -= 360
        elif angle_distance < -180:
            angle_distance += 360

        if angle_distance > MyRobot.THRESHOLD_ANGLE:
            self.robot_drive.arcadeDrive(.5, 0)
        elif angle_distance < -MyRobot.THRESHOLD_ANGLE:
            self.robot_drive.arcadeDrive(-.5, 0)
        elif distance > MyRobot.THRESHOLD_DISTANCE:
            self.robot_drive.arcadeDrive(0, -.5)
        else:
            self.robot_drive.arcadeDrive(0, 0)


    def calculate_odometry(self):

        # get how much time has passed
        dt = self.getPeriod()

        left_angular_pos = self.get_left_angular_pos()
        right_angular_pos = self.get_right_angular_pos()

        # calculate angular velocities of the wheels
        left_angular_vel = (left_angular_pos - self.prev_left_angular_pos) / dt
        right_angular_vel = (right_angular_pos - self.prev_right_angular_pos) / dt

        # calculate the velocity of the robot
        vel = (left_angular_vel + right_angular_vel) * MyRobot.WHEEL_RADIUS / 2

        # calculate angular velocity and use it to calculate the new heading of the robot
        angular_vel = (left_angular_vel - right_angular_vel) * MyRobot.WHEEL_RADIUS / MyRobot.DRIVE_BASE
        self.angle += angular_vel * dt

        # Update the robot's position
        self.x_pos += vel * math.sin(self.angle) * dt
        self.y_pos += vel * math.cos(self.angle) * dt

        # save these values so we can take the difference between them. We want to measure
        # the difference in order to calculate the angular velocity.
        self.prev_left_angular_pos = left_angular_pos
        self.prev_right_angular_pos = right_angular_pos


    def reset_odometry(self):
        self.fl_motor.setQuadraturePosition(0, 0)
        self.br_motor.setQuadraturePosition(0, 0)
        self.x_pos = 0
        self.y_pos = 0
        self.angle = 0
        self.prev_left_angular_pos = self.get_left_angular_pos()
        self.prev_right_angular_pos = self.get_right_angular_pos()


    def get_angle(self):
        '''Returns an angle between -180 and 180'''
        angle_degrees = (self.angle % (math.pi * 2)) * (180 / math.pi)
        if angle_degrees < -180:
            return angle_degrees + 360
        elif angle_degrees > 180:
            return angle_degrees - 360
        else:
            return angle_degrees

    def get_position(self):
        return self.x_pos / 12, self.y_pos / 12

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