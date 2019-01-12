#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
from networktables import NetworkTables
from networktables.util import ntproperty


class MyRobot(wpilib.TimedRobot):

    servo_value_max = ntproperty('/servo_max', 1)
    servo_value_min = ntproperty('/servo_min', 0)

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


        self.elevator_motor = ctre.wpi_talonsrx.WPI_TalonSRX(4)
        self.elevator_follower = ctre.wpi_talonsrx.WPI_TalonSRX(10)
        self.elevator_follower.follow(self.elevator_motor)

        self.chomp_relay = wpilib.Relay(3)

        self.servo = wpilib.Servo(2)

        self.last_button_value = False

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

        # elevator teleop
        elevator_speed = -0.1

        # A button
        if self.joystick.getRawButton(1):
            # go down
            elevator_speed = 0.45
        # Y Button
        elif self.joystick.getRawButton(4):
            # go up
            elevator_speed = -.6

        self.elevator_motor.set(elevator_speed)

        current_value = self.joystick.getRawButton(3)
        if current_value and not self.last_button_value:

            print("toggled!!!")
            if self.chomp_relay.get() == wpilib.Relay.Value.kOff:
                self.chomp_relay.set(wpilib.Relay.Value.kOn)
            else:
                self.chomp_relay.set(wpilib.Relay.Value.kOff)

        self.last_button_value = current_value

        # left bumper
        if self.joystick.getRawButton(5):
            self.servo.set(1)
        # right bumper
        if self.joystick.getRawButton(6):
            self.servo.set(0)



        print(self.servo.get())





if __name__ == "__main__":
    wpilib.run(MyRobot)