#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
import math
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




    ticks_per_rev_left = ntproperty('/encoders/ticks_per_rev_left', -1000)
    ticks_per_rev_right = ntproperty('/encoders/ticks_per_rev', 1440)
    max_speed = ntproperty('/encoders/max_speed', 5)

    p_left = ntproperty('/encoders/p_left', 1)
    i_left = ntproperty('/encoders/i_left', 0)
    d_left = ntproperty('/encoders/d_left', 0)

    p_right = ntproperty('/encoders/p_right', 1)
    i_right = ntproperty('/encoders/i_right', 0)
    d_right = ntproperty('/encoders/d_right', 0)

    wheel_diameter = ntproperty('/encoders/wheel_diameter', 6)




    image_center = 15

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.rev_per_ft = 12 / (math.pi * self.wheel_diameter)

        self.ticks_per_ft_left = self.rev_per_ft * self.ticks_per_rev_left
        self.ticks_per_ft_right = self.rev_per_ft * self.ticks_per_rev_right


        self.br_motor = ctre.wpi_talonsrx.WPI_TalonSRX(1)
        self.bl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(2)
        self.fl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(5)
        self.fr_motor = ctre.wpi_talonsrx.WPI_TalonSRX(7)

        self.fr_motor.setInverted(True)
        self.br_motor.setInverted(True)

        self.br_motor.follow(self.fr_motor)
        self.bl_motor.follow(self.fl_motor)


        self.fl_motor.config_kP(0, self.p_left, 0)
        self.fl_motor.config_kI(0, self.i_left, 0)
        self.fl_motor.config_kD(0, self.d_left, 0)

        self.fr_motor.config_kP(0, self.p_right, 0)
        self.fr_motor.config_kI(0, self.i_right, 0)
        self.fr_motor.config_kD(0, self.d_right, 0)



        self.fr_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
        self.fr_motor.setSensorPhase(False)


        self.fl_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)

        #self.robot_drive = wpilib.RobotDrive(self.fl_motor, self.bl_motor, self.fr_motor, self.br_motor)

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

        elif key == '/encoders/p_left':
            self.fl_motor.config_kP(0, self.p_left, 0)
        elif key == '/encoders/i_left':
            self.fl_motor.config_kI(0, self.i_left, 0)
        elif key == '/encoders/d_left':
            self.fl_motor.config_kD(0, self.d_left, 0)

        elif key == '/encoders/p_right':
            self.fr_motor.config_kP(0, self.p_right, 0)
        elif key == '/encoders/i_right':
            self.fr_motor.config_kI(0, self.i_right, 0)
        elif key == '/encoders/d_right':
            self.fr_motor.config_kD(0, self.d_right, 0)

    def pid_source(self):
        return self.target_x

    def pid_output(self, output):
        # make something happen here
        self.turn_rate = -output

    def get_left_angular_pos(self):
      encoder_value = self.fl_motor.getQuadraturePosition()
      ticks_per_turn = -1000
      return 2 * math.pi * encoder_value / ticks_per_turn


    def get_right_angular_pos(self):
        encoder_value = self.br_motor.getQuadraturePosition()
        ticks_per_turn = 1440
        return 2 * math.pi * encoder_value / ticks_per_turn

    def to_motor_speed(self, ft_per_second, ticks_per_rev):
        ticks_per_ft = ticks_per_rev * self.rev_per_ft
        ticks_per_sec = ft_per_second* ticks_per_ft
        return ticks_per_sec * .1


    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        #self.robot_drive.arcadeDrive(0, 0)
        pass

    def teleopInit(self):
        limelight_table = NetworkTables.getTable("limelight")
        limelight_table.putNumber('ledMode', 1)
        self.pid.setSetpoint(MyRobot.image_center)
        self.pid.enable()


    def teleopPeriodic(self):
        """This function is called periodically during operator control."""

        left_speed = self.joystick.getRawAxis(1) * self.max_speed
        left_motor_speed = self.to_motor_speed(left_speed, self.ticks_per_rev_left)

        print('selected sensor velocity', left_motor_speed, self.fl_motor.getSelectedSensorVelocity(0))

        self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, left_motor_speed)

        right_speed = -self.joystick.getRawAxis(5) * self.max_speed
        right_motor_speed = self.to_motor_speed(right_speed, self.ticks_per_rev_right)
        self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, right_motor_speed)

        return

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