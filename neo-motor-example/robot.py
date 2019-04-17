#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import rev


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.rev_motor = rev.CANSparkMax(25, rev.MotorType.kBrushless)
        self.joystick = wpilib.Joystick(0)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.rev_motor.getPIDController().setP(1)
        self.rev_motor.getPIDController().setI(0)
        self.rev_motor.getPIDController().setD(0)

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.rev_motor.set(-self.joystick.getRawAxis(1))
        print(self.rev_motor.getEncoder().getPosition())
        

if __name__ == "__main__":
    wpilib.run(MyRobot)