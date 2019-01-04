#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
from networktables import NetworkTables
from networktables.util import ntproperty



class MyRobot(wpilib.TimedRobot):

    target_found = ntproperty('/vision/target_found', False)
    target_size = ntproperty('/vision/target_size', 0)
    target_x = ntproperty('/vision/target_x', 0)
    target_y = ntproperty('/vision/target_y', 0)


    P = ntproperty('/PID/P', .02)
    I = ntproperty('/PID/I', 0)
    D = ntproperty('/PID/D', 0)


    image_center = 15

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

        self.timer = wpilib.Timer()

        self.last_seen = 0

        wpilib.CameraServer.launch('vision.py:main')

        limelight_table = NetworkTables.getTable("limelight")
        limelight_table.putNumber('ledMode', 1)

        self.pid = wpilib.PIDController(self.P, self.I, self.D, self.pid_source, self.pid_output)
        self.pid.setOutputRange(-1, 1)
        self.pid.setInputRange(-15, 45)

        self.turn_rate = 0

        NetworkTables.addEntryListener(self.entry_listener)

    def entry_listener(self, key, value, is_new):
        if key == '/vision/target_found':
            print('target status changed to ', value)

    def pid_source(self):
        return self.target_x

    def pid_output(self, output):
        # make something happen here
        self.turn_rate = -output

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.robot_drive.arcadeDrive(0, 0)

    def teleopInit(self):
        limelight_table = NetworkTables.getTable("limelight")
        limelight_table.putNumber('ledMode', 1)
        self.pid.setSetpoint(MyRobot.image_center)
        self.pid.enable()


    def teleopPeriodic(self):
        """This function is called periodically during operator control."""


        if self.target_found:
            self.last_seen = self.timer.getMsClock()



        if (self.pid.getP() != self.P):
            self.pid.setP(self.P)

        if (self.pid.getI() != self.I):
            self.pid.setI(self.I)

        if (self.pid.getD() != self.D):
            self.pid.setD(self.D)

        if self.joystick.getRawButton(1):
            if (self.timer.getMsClock() - self.last_seen) > 500:
                self.pid.reset()

            self.pid.enable()
            if self.target_found:
                self.robot_drive.arcadeDrive(self.turn_rate, self.joystick.getY())
        else:
            self.pid.reset()
            # self.pid.disable()
            self.robot_drive.arcadeDrive(self.joystick.getX(), self.joystick.getY())



if __name__ == "__main__":
    wpilib.run(MyRobot)