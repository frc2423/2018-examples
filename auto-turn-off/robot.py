#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        self.BRmotor = ctre.wpi_talonsrx.WPI_TalonSRX(40)
        self.BLmotor = ctre.wpi_talonsrx.WPI_TalonSRX(50)
        self.FRmotor = ctre.wpi_talonsrx.WPI_TalonSRX(10)
        self.FLmotor = ctre.wpi_talonsrx.WPI_TalonSRX(30)

        self.servo = wpilib.Servo(8)


        self.timer = wpilib.Timer()
        self.robot_drive = wpilib.RobotDrive(self.FLmotor, self.BLmotor, self.FRmotor, self.BRmotor)
        self.joystick = wpilib.Joystick(0)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.robot_drive.arcadeDrive(0, 0)

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
         self.timer.start()
  
    def teleopPeriodic(self):
        self.robot_drive.arcadeDrive(self.joystick.getY(), self.joystick.getX())
        #Robot turn off
       
        if self.timer.get() > 10:
            self.servo.set(0)
        

if __name__ == "__main__":
    wpilib.run(MyRobot)



    