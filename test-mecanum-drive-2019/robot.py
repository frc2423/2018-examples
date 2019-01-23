#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
from networktables import NetworkTables
from networktables.util import ntproperty
import math
from mecanum import driveCartesian
import navx


class MyRobot(wpilib.TimedRobot):

    TIMEOUT_MS = 30

    servo_position = ntproperty('/Servo/Value', .5)
    servo_offset1 = ntproperty('/Servo/Offset1', 0)
    servo_offset2 = ntproperty('/Servo/Offset2', 0)
    arm_up = ntproperty('/Servo/ArmUp', 0)
    arm_down = ntproperty('/Servo/ArmDown', 0)
    #liftforball = ntproperty('/Servo/ArmforBall', 0.5)

    ticks_per_rev = ntproperty('/encoders/ticks_per_rev', 1440)
    wheel_diameter = ntproperty('/encoders/wheel_diameter', 6)
    max_speed = ntproperty('/encoders/max_speed', 12)
    wheel_diameter = ntproperty('/encoders/wheel_diameter', 6)

    ticks_per_rev_fl = ntproperty('/encoders/ticks_per_rev_fl', 1440) # done
    ticks_per_rev_bl = ntproperty('/encoders/ticks_per_rev_bl', 1440) # done
    ticks_per_rev_fr = ntproperty('/encoders/ticks_per_rev_fr', 1440) # done
    ticks_per_rev_br = ntproperty('/encoders/ticks_per_rev_br', 1440) # done

    p_fl = ntproperty('/encoders/p_fl', .5)
    i_fl = ntproperty('/encoders/i_fl', .001)
    d_fl = ntproperty('/encoders/d_fl', 0)
    f_fl = ntproperty('/encoders/f_fl', .7)

    p_bl = ntproperty('/encoders/p_bl', .5)
    i_bl = ntproperty('/encoders/i_bl', .001)
    d_bl = ntproperty('/encoders/d_bl', 0)
    f_bl = ntproperty('/encoders/f_bl', 0.7)

    p_fr = ntproperty('/encoders/p_fr', .5)
    i_fr = ntproperty('/encoders/i_fr', 0.001)
    d_fr = ntproperty('/encoders/d_fr', 0)
    f_fr = ntproperty('/encoders/f_fr', 0.7)

    p_br = ntproperty('/encoders/p_br', .5)
    i_br = ntproperty('/encoders/i_br', 0.001)
    d_br = ntproperty('/encoders/d_br', 0)
    f_br = ntproperty('/encoders/f_br', 0.7)



    def robotInit(self):

        self.rev_per_ft = 12 / (math.pi * self.wheel_diameter)

        self.br_motor = ctre.wpi_talonsrx.WPI_TalonSRX(40)
        self.bl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(50)
        self.fr_motor = ctre.wpi_talonsrx.WPI_TalonSRX(10)
        self.fl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(30)

        self.br_motor.setInverted(True)
        self.fr_motor.setInverted(True)

        self.fl_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)
        self.bl_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)
        self.fr_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)
        self.br_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)

        self.fr_motor.setSensorPhase(False)  # Reverse negative encoder values
        self.br_motor.setSensorPhase(True)

        self.fl_motor.config_kP(0, self.p_fl, MyRobot.TIMEOUT_MS)
        self.fl_motor.config_kI(0, self.i_fl, MyRobot.TIMEOUT_MS)
        self.fl_motor.config_kD(0, self.d_fl, MyRobot.TIMEOUT_MS)
        self.fl_motor.config_kF(0, self.f_fl, MyRobot.TIMEOUT_MS)

        self.bl_motor.config_kP(0, self.p_bl, MyRobot.TIMEOUT_MS)
        self.bl_motor.config_kI(0, self.i_bl, MyRobot.TIMEOUT_MS)
        self.bl_motor.config_kD(0, self.d_bl, MyRobot.TIMEOUT_MS)
        self.bl_motor.config_kF(0, self.f_bl, MyRobot.TIMEOUT_MS)

        self.fr_motor.config_kP(0, self.p_fr, MyRobot.TIMEOUT_MS)
        self.fr_motor.config_kI(0, self.i_fr, MyRobot.TIMEOUT_MS)
        self.fr_motor.config_kD(0, self.d_fr, MyRobot.TIMEOUT_MS)
        self.fr_motor.config_kF(0, self.f_fr, MyRobot.TIMEOUT_MS)

        self.br_motor.config_kP(0, self.p_br, MyRobot.TIMEOUT_MS)
        self.br_motor.config_kI(0, self.i_br, MyRobot.TIMEOUT_MS)
        self.br_motor.config_kD(0, self.d_br, MyRobot.TIMEOUT_MS)
        self.br_motor.config_kF(0, self.f_br, MyRobot.TIMEOUT_MS)



        self.spinman = ctre.wpi_talonsrx.WPI_TalonSRX(5)

        self.littlearms1 = wpilib.Servo(7)
        self.littlearms2 = wpilib.Servo(8)

        self.joystick = wpilib.Joystick(0)

        NetworkTables.addEntryListener(self.entry_listener)

        self.use_pid = False
        self.prev_pid_toggle_btn_value = False

        self.navx = navx.AHRS.create_spi()



        self.desired_angle = 0
        self.pid_turn_rate = 0

        self.turn_rate_pid = wpilib.PIDController(1, 0, 0, self.get_normalized_angle, self.set_pid_turn_rate)
        #self.turn_rate_pid.
        #self.turn_rate_pid.
        self.turn_rate_pid.setInputRange(-180, 180)
        self.turn_rate_pid.setOutputRange(-.5, .5)
        self.turn_rate_pid.setContinuous(True)


    def set_pid_turn_rate(self, turn_rate):
        self.pid_turn_rate = turn_rate
        print('turn_rate:', turn_rate)


    def get_normalized_angle(self):

        angle = self.navx.getAngle() % 360

        if angle > 180:
            return angle - 360
        elif angle < -180:
            return angle + 360
        else:
            return angle

    def entry_listener(self, key, value, is_new):

        if key == '/encoders/p_fl':
            self.fl_motor.config_kP(0, self.p_fl, 0)
        elif key == '/encoders/i_fl':
            self.fl_motor.config_kI(0, self.i_fl, 0)
        elif key == '/encoders/d_fl':
            self.fl_motor.config_kD(0, self.d_fl, 0)
        elif key == '/encoders/f_fl':
            self.fl_motor.config_kF(0, self.f_fl, 0)





    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        #self.robot_drive.mecanumDrive_Cartesian(1, 1, 0, 0)
        pass

    def autonomousPeriodic(self):
        pass


    def teleopInit(self):
        self.desired_angle = 0
        self.navx.reset()
        self.turn_rate_pid.enable()

    def on_pid_toggle(self):

        pid_toggle_btn_value = self.joystick.getRawButton(4)

        if not self.prev_pid_toggle_btn_value and pid_toggle_btn_value:
            self.use_pid = not self.use_pid

        self.prev_pid_toggle_btn_value = pid_toggle_btn_value





    def teleopPeriodic(self):

        # max turn rate = 360 degrees / s
        max_turn_rate = 360
        dt = self.getPeriod()

        self.desired_angle = self.desired_angle + self.deadzone(self.joystick.getRawAxis(0)) * max_turn_rate * dt
        self.turn_rate_pid.setSetpoint(self.desired_angle)

        print('error:', self.turn_rate_pid.getError())
        #print('desired angle:', self.desired_angle)


        #x_speed = self.deadzone(-self.joystick.getRawAxis(1), .15) * self.max_speed

        self.on_pid_toggle()

        if self.joystick.getRawButton(2):
            x_speed = -.4
            y_speed = 0
            z_speed = 0
        elif self.joystick.getRawButton(3):
            x_speed = .4
            y_speed = 0
            z_speed = 0
        else:
            x_speed = self.deadzone(-self.joystick.getRawAxis(0), .15)
            y_speed = self.deadzone(-self.joystick.getRawAxis(1), .15)
            z_speed = self.pid_turn_rate #self.deadzone(-self.joystick.getRawAxis(4), .15)

        fl, bl, fr, br = driveCartesian(-x_speed, y_speed, -z_speed, self.navx.getAngle())

        if not self.use_pid:
            self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fl)
            self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, bl)
            self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fr)
            self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, br)


        else:
            fl_speed = self.to_motor_speed(fl * self.max_speed, self.ticks_per_rev)
            bl_speed = self.to_motor_speed(bl * self.max_speed, self.ticks_per_rev)
            fr_speed = self.to_motor_speed(fr * self.max_speed, self.ticks_per_rev)
            br_speed = self.to_motor_speed(br * self.max_speed, self.ticks_per_rev)

            self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, fl_speed)
            self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, bl_speed)
            self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, fr_speed)
            self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, br_speed)


        #left_speed = self.deadzone(-self.joystick.getRawAxis(1), .15) * self.max_speed
        #left_motor_speed = self.to_motor_speed(left_speed, self.ticks_per_rev)



        #print('selected sensor velocity', left_motor_speed, self.fl_motor.getSelectedSensorVelocity(0))

        #self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, left_motor_speed)
        #self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, left_motor_speed)

        #right_speed = self.deadzone(-self.joystick.getRawAxis(5), .15) * self.max_speed
        #right_motor_speed = self.to_motor_speed(right_speed, self.ticks_per_rev)

        #self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, right_motor_speed)
        #self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, right_motor_speed)

        '''
        print('encoder counts:',
              self.fl_motor.getQuadraturePosition(),
              self.bl_motor.getQuadraturePosition(),
              self.fr_motor.getQuadraturePosition(),
              self.br_motor.getQuadraturePosition())
              
              

        self.teleop_arms()
        '''

    def regular_mec_drive(self):
        x = self.joystick.getRawAxis(0)
        y = self.joystick.getRawAxis(1)
        rot = self.joystick.getRawAxis(4)

        self.robot_drive.mecanumDrive_Cartesian(self.dead_zone(x), self.dead_zone(y), self.dead_zone(rot), 0)

    def teleop_arms(self):
        if self.joystick.getRawAxis(2) > .2:
            self.spinman.set(-self.joystick.getRawAxis(2) * .5)
        elif self.joystick.getRawAxis(3) > .2:
            self.spinman.set(self.joystick.getRawAxis(3) * .5)
        else:
            self.spinman.set(0)

        #if self.joystick.getRawButton(4):
         #   self.armsup = not self.armsup
        #if self.armsup:
         #   self.littlearms1.set(self.liftforball + self.servo_offset1)
          #  self.littlearms2.set(self.liftforball + self.servo_offset2)


        if self.joystick.getRawButton(2):
            self.littlearms1.set(self.arm_up + self.servo_offset1)
            self.littlearms2.set(self.arm_up + self.servo_offset2)
        else:
            self.littlearms1.set(self.arm_down + self.servo_offset1)
            self.littlearms2.set(self.arm_down + self.servo_offset2)

        print("Lil Arms 1: ",self.littlearms1.get(), "Lil Arms 2:",self.littlearms2.get())


    def deadzone(self, value, min = .1):
        if -min < value < min:
            return 0
        else:
            scaled_value = (abs(value) - min) / (1 - min)
            return math.copysign(scaled_value, value)

    def to_motor_speed(self, ft_per_second, ticks_per_rev):
        ticks_per_ft = ticks_per_rev * self.rev_per_ft
        ticks_per_sec = ft_per_second * ticks_per_ft
        return ticks_per_sec * .1

if __name__ == "__main__":
    wpilib.run(MyRobot)