#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
from networktables import NetworkTables
from networktables.util import ntproperty


class MyRobot(wpilib.TimedRobot):

    servo_position = ntproperty('/Servo/Value', .5)
    servo_offset = ntproperty('/Servo/Offset', 0)

    def robotInit(self):
        self.BRmotor = ctre.wpi_talonsrx.WPI_TalonSRX(40)
        self.BLmotor = ctre.wpi_talonsrx.WPI_TalonSRX(50)
        self.FRmotor = ctre.wpi_talonsrx.WPI_TalonSRX(10)
        self.FLmotor = ctre.wpi_talonsrx.WPI_TalonSRX(30)

        self.BRmotor.setInverted(True)
        self.FRmotor.setInverted(True)

        self.spinman = ctre.wpi_talonsrx.WPI_TalonSRX(5)
        self.timer = wpilib.Timer()

        self.littlearms1 = wpilib.Servo(7)
        self.littlearms2 = wpilib.Servo(8)

        self.robot_drive = wpilib.RobotDrive(self.FLmotor, self.BLmotor, self.FRmotor, self.BRmotor)
        self.joystick = wpilib.Joystick(0)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.robot_drive.mecanumDrive_Cartesian(1, 1, 0, 0)

    def autonomousPeriodic(self):
        pass

    def teleopPeriodic(self):
        x = self.joystick.getRawAxis(0)
        y = self.joystick.getRawAxis(1)
        rot = self.joystick.getRawAxis(4)
        def dead_zone(values):
            if values < 0.1 and values > -0.1:
                return 0
            else:
                return values
        self.robot_drive.mecanumDrive_Cartesian(dead_zone(x), dead_zone(y), dead_zone(rot), 0)

        if self.joystick.getRawAxis(2) > .2:
            self.spinman.set(-self.joystick.getRawAxis(2) * .5)
        elif self.joystick.getRawAxis(3) > .2:
            self.spinman.set(self.joystick.getRawAxis(3) * .5)
        else:
            self.spinman.set(0)

        self.littlearms1.set(self.servo_position)
        self.littlearms2.set(self.servo_position)

        #if self.joystick.getRawButton(5):
         #   self.littlearms1.set(1)
          #  self.littlearms2.set(1)
        #elif self.joystick.getRawButton(6):
         #   self.littlearms1.set(0)
         #   self.littlearms2.set(0)
        #else:
         #   self.littlearms1.set(0.5)
          #  self.littlearms2.set(0.5)

        print("Lil Arms 1: ",self.littlearms1.get(), "Lil Arms 2:",self.littlearms2.get())

if __name__ == "__main__":
    wpilib.run(MyRobot)