#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
from ctre import _impl
import math
from networktables import NetworkTables
from networktables.util import ntproperty
from arcade_drive import arcade_drive



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

    p_velocity_left = ntproperty('/encoders/p_velocity_left', 1)
    i_velocity_left = ntproperty('/encoders/i_velocity_left', .01)
    d_velocity_left = ntproperty('/encoders/d_velocity_left', 0)
    f_velocity_left = ntproperty('/encoders/f_velocity_left', 1.5)

    p_velocity_right = ntproperty('/encoders/p_velocity_right', 1)
    i_velocity_right = ntproperty('/encoders/i_velocity_right', 0.005)
    d_velocity_right = ntproperty('/encoders/d_velocity_right', 0)
    f_velocity_right = ntproperty('/encoders/f_velocity_right', 1.4)

    p_position_left = ntproperty('/encoders/p_position_left', 1)
    i_position_left = ntproperty('/encoders/i_position_left', .01)
    d_position_left = ntproperty('/encoders/d_position_left', 0)
    f_position_left = ntproperty('/encoders/f_position_left', 1.5)

    p_position_right = ntproperty('/encoders/p_position_right', 1)
    i_position_right = ntproperty('/encoders/i_position_right', 0.005)
    d_position_right = ntproperty('/encoders/d_position_right', 0)
    f_position_right = ntproperty('/encoders/f_position_right', 1.4)

    wheel_diameter = ntproperty('/encoders/wheel_diameter', 6)

    position_mode_distance = ntproperty('/position/max_distance', 1)

    image_center = 15

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.rev_per_ft = 12 / (math.pi * self.wheel_diameter)

        self.ticks_per_ft_left = self.rev_per_ft * self.ticks_per_rev_left
        self.ticks_per_ft_right = self.rev_per_ft * self.ticks_per_rev_right

        self.TIMEOUT_MS = 30

        self.BUTTON_A = 1


        self.br_motor = ctre.wpi_talonsrx.WPI_TalonSRX(7)
        self.bl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(2)
        self.fl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(5)
        self.fr_motor = ctre.wpi_talonsrx.WPI_TalonSRX(1)

        self.fr_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, self.TIMEOUT_MS)
        self.fl_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, self.TIMEOUT_MS)

        # maybe this can be used to scale encoder values to ft?
        #self.fl_motor.configSelectedFeedbackCoefficient(2, 0, 0)

        self.fl_motor.setSensorPhase(True)
        self.fr_motor.setSensorPhase(True)

        self.br_motor.follow(self.fr_motor)
        self.bl_motor.follow(self.fl_motor)

        self.fr_motor.setInverted(True)
        self.br_motor.setInverted(True)


        self.fl_motor.setSensorPhase(True)
        self.fr_motor.setSensorPhase(True)

        #initial positions for position modes
        self.initial_position_left = 0
        self.initial_position_right = 0

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

        self.position_mode_toggle = False

        NetworkTables.addEntryListener(self.entry_listener)

    def entry_listener(self, key: str, value, is_new):
        if("encoders" in key):
            print(f"key change.  {key}={value}")
            self.set_position_pid()
        if key == '/vision/target_found':
            print('target status changed to ', value)

    def set_velocity_pid(self):
        print('setting velocity pid')
        print(f"Left: p={self.p_velocity_left}, i={self.i_velocity_left}, d={self.d_velocity_left}, f={self.f_velocity_left}")
        print(f"Right: p={self.p_velocity_right}, i={self.i_velocity_right}, d={self.d_velocity_right}, f={self.f_velocity_right}")
        print()
        self.fl_motor.config_kP(0, self.p_velocity_left, self.TIMEOUT_MS)
        self.fl_motor.config_kI(0, self.i_velocity_left, self.TIMEOUT_MS)
        self.fl_motor.config_kD(0, self.d_velocity_left, self.TIMEOUT_MS)
        self.fl_motor.config_kF(0, self.f_velocity_left, self.TIMEOUT_MS)

        self.fr_motor.config_kP(0, self.p_velocity_right, self.TIMEOUT_MS)
        self.fr_motor.config_kI(0, self.i_velocity_right, self.TIMEOUT_MS)
        self.fr_motor.config_kD(0, self.d_velocity_right, self.TIMEOUT_MS)
        self.fr_motor.config_kF(0, self.f_velocity_right, self.TIMEOUT_MS)

    def set_position_pid(self):
        print('setting position pid')
        print(f"Left: p={self.p_position_left}, i={self.i_position_left}, d={self.d_position_left}, f={self.f_position_left}")
        print(f"Right: p={self.p_position_right}, i={self.i_position_right}, d={self.d_position_right}, f={self.f_position_right}")
        print()

        self.fl_motor.config_kP(0, self.p_position_left, self.TIMEOUT_MS)
        self.fl_motor.config_kI(0, self.i_position_left, self.TIMEOUT_MS)
        self.fl_motor.config_kD(0, self.d_position_left, self.TIMEOUT_MS)
        self.fl_motor.config_kF(0, self.f_position_left, self.TIMEOUT_MS)

        self.fr_motor.config_kP(0, self.p_position_right, self.TIMEOUT_MS)
        self.fr_motor.config_kI(0, self.i_position_right, self.TIMEOUT_MS)
        self.fr_motor.config_kD(0, self.d_position_right, self.TIMEOUT_MS)
        self.fr_motor.config_kF(0, self.f_position_right, self.TIMEOUT_MS)

    def pid_source(self):
        return self.target_x

    def pid_output(self, output):
        # make something happen here
        self.turn_rate = -output

    def get_left_angular_pos(self):
      encoder_value = self.fl_motor.getQuadraturePosition()
      ticks_per_turn = self.ticks_per_rev_left
      return self.wheel_diameter * math.pi * encoder_value / ticks_per_turn


    def get_right_angular_pos(self):
        """this is what it does"""
        encoder_value = self.fr_motor.getQuadraturePosition()
        ticks_per_turn = self.ticks_per_rev_right
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
        self.position_mode_toggle = False
        self.set_velocity_pid()


    def teleopPeriodic(self):

        """This function is called periodically during operator control."""

        if self.position_mode_toggle and self.joystick.getRawButton(self.BUTTON_A):
            self.position_mode()
        elif self.joystick.getRawButton(self.BUTTON_A):
            print("Position mode transition")
            self.position_mode_toggle = True

            self.fl_motor.setIntegralAccumulator(0, 0, 0)
            self.fr_motor.setIntegralAccumulator(0, 0, 0)
            stick_displacement_left = self.deadzone(self.joystick.getRawAxis(1)) * self.ticks_per_ft_left
            stick_displacement_right = self.deadzone(self.joystick.getRawAxis(1)) * self.ticks_per_ft_right
            self.initial_position_right = self.fr_motor.getQuadraturePosition() + stick_displacement_right #self.get_right_angular_pos()
            self.initial_position_left = -(self.fl_motor.getQuadraturePosition() + stick_displacement_left) #self.get_left_angular_pos()
            self.set_position_pid()
            print(f"position mode     Left: {self.initial_position_left}    Right: {self.initial_position_right}" )
        elif self.position_mode_toggle:
            print("Speed mode transition")
            if abs(self.joystick.getRawAxis(1)) < .1:
                self.position_mode_toggle = False
                #self.set_velocity_pid()

        else:
            self.speed_mode()

        #print(f"encoder left: {self.fl_motor.getQuadraturePosition()}    encoder right: {self.fr_motor.getQuadraturePosition()}")

        #self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Position, self.initial_position_left + self.position_mode_distance)
        #self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Position, self.initial_position_right + self.position_mode_distance)
        if self.target_found:
            self.last_seen = self.timer.getMsClock()

    def speed_mode(self):
        left, right = arcade_drive(self.joystick.getRawAxis(1), -self.joystick.getRawAxis(0))

        left_speed = self.deadzone(left, .15) * self.max_speed
        left_motor_speed = self.to_motor_speed(left_speed, self.ticks_per_rev_left)
        self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, left_motor_speed)

        right_speed = self.deadzone(-right, .15) * self.max_speed
        right_motor_speed = self.to_motor_speed(right_speed, self.ticks_per_rev_right)
        self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, right_motor_speed)

        #print('selected sensor velocity', left_motor_speed, self.fl_motor.getSelectedSensorVelocity(0))

        return

    def position_mode(self):


        displacement_right = self.deadzone(self.joystick.getRawAxis(1)) * self.ticks_per_ft_right * self.position_mode_distance
        displacement_left = self.deadzone(self.joystick.getRawAxis(1)) * self.ticks_per_ft_left * self.position_mode_distance

        # print(f'displacement: left: {displacement_left} right: {displacement_right}' )
        # print(f'position: left{self.fl_motor.getQuadraturePosition()} right: {self.fr_motor.getQuadraturePosition()}'  )
        # print(f'initial left: {self.initial_position_left} right: {self.initial_position_right}')
        # print(f'target: left: {self.initial_position_left + displacement_left} right: {self.initial_position_right - displacement_right}')
        #
        # print('------------------------------------------------------------------------------------')

        self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Position, self.initial_position_left + displacement_left)
        self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Position, self.initial_position_right - displacement_right)

        #print(f"displacement left: {displacement_left}    displacement right: {displacement_right}    initial left: {self.initial_position_left}    initial_right: {self.initial_position_right}")


    def deadzone(self, value, min = .1):
        if -min < value < min:
            return 0
        else:
            scaled_value = (abs(value) - min) / (1 - min)
            return math.copysign(scaled_value, value)



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