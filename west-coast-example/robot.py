#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import rev
from networktables import NetworkTables
from networktables.util import ntproperty
import math
from wpilib.drive import DifferentialDrive

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.lt_motor = rev.CANSparkMax(2, rev.MotorType.kBrushless)
        self.lf_motor = rev.CANSparkMax(3, rev.MotorType.kBrushless)
        self.lb_motor = rev.CANSparkMax(1, rev.MotorType.kBrushless)
        self.rt_motor = rev.CANSparkMax(5, rev.MotorType.kBrushless)
        self.rf_motor = rev.CANSparkMax(4, rev.MotorType.kBrushless)
        self.rb_motor = rev.CANSparkMax(6, rev.MotorType.kBrushless)
        
        self.left = wpilib.SpeedControllerGroup(self.lt_motor, self.lf_motor, self.lb_motor)
        self.right = wpilib.SpeedControllerGroup(self.rt_motor, self.rf_motor, self.rb_motor)

        self.drive = DifferentialDrive(self.left, self.right)

        self.lt_motor.setInverted(True)
        self.rt_motor.setInverted(True)

        self.joystick = wpilib.Joystick(0)

        self.previous_button = False

        self.gear_switcher = wpilib.DoubleSolenoid(0, 1)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        #self.rev_motor.set(-self.joystick.getRawAxis(1))
        #print(self.rev_motor.getEncoder().getVelocity())
        self.drive.tankDrive(-self.joystick.getRawAxis(1), -self.joystick.getRawAxis(5))

        #self.drive.arcadeDrive(self.joystick.getRawAxis(2), -self.joystick.getRawAxis(1))

        current_button = self.joystick.getRawButton(1)

        clicked = self.previous_button and not current_button

        if clicked:
            if self.gear_switcher.get() == wpilib.DoubleSolenoid.Value.kForward:
                self.gear_switcher.set(wpilib.DoubleSolenoid.Value.kReverse)
            else:
                self.gear_switcher.set(wpilib.DoubleSolenoid.Value.kForward)

        self.previous_button = current_button
    
    def deadzone(self, value, min = .2):
        if -min < value < min:
            return 0
        else:
            scaled_value = (abs(value) - min) / (1 - min)
            return math.copysign(scaled_value, value)
        

if __name__ == "__main__":
    wpilib.run(MyRobot)