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

        # line sensors 0 is far left, 3 is far right
        self.line_sensor0 = wpilib.DigitalInput(4)
        self.line_sensor1 = wpilib.DigitalInput(1)
        self.line_sensor2 = wpilib.DigitalInput(0)
        self.line_sensor3 = wpilib.DigitalInput(3)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.robot_drive.arcadeDrive(0, 0)



    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        speed = self.joystick.getY()

        SEES_LINE = True
        print('sensor value:', self.line_sensor0.get(), self.line_sensor1.get(), self.line_sensor2.get(), self.line_sensor3.get())
        #self.line_sensor0.
        if self.joystick.getTrigger():
           if self.line_sensor0.get() == SEES_LINE:
               self.robot_drive.arcadeDrive(speed, 0)
           elif self.line_sensor1.get() == SEES_LINE:
               self.robot_drive.arcadeDrive(speed , speed* .8)
           elif self.line_sensor2.get() == SEES_LINE:
               self.robot_drive.arcadeDrive(-speed, speed * .8)
           elif self.line_sensor3.get() == SEES_LINE:
               self.robot_drive.arcadeDrive(-speed, 0)
           else:
               self.robot_drive.arcadeDrive(0, speed * .8)
        else:
            self.robot_drive.arcadeDrive(self.joystick.getX(), speed)



if __name__ == "__main__":
    wpilib.run(MyRobot)