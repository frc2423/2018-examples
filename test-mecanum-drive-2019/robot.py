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
from state_machine import State_Machine


class MyRobot(wpilib.TimedRobot):
    TIMEOUT_MS = 30

    p = ntproperty('/encoders/p_fl', .5)
    i = ntproperty('/encoders/i_fl', .001)
    d = ntproperty('/encoders/d_fl', 0.0)
    f = ntproperty('/encoders/f_fl', .7)

    displacement_multiplier = ntproperty("/encoders/displacement_multiplier", 500)


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

    def setEncoderPids(self):
        print("setting encoder PIDs")

        self.fl_motor.config_kP(0, self.p, MyRobot.TIMEOUT_MS)
        self.fl_motor.config_kI(0, self.i, MyRobot.TIMEOUT_MS)
        self.fl_motor.config_kD(0, self.d, MyRobot.TIMEOUT_MS)
        self.fl_motor.config_kF(0, self.f, MyRobot.TIMEOUT_MS)

        self.bl_motor.config_kP(0, self.p, MyRobot.TIMEOUT_MS)
        self.bl_motor.config_kI(0, self.i, MyRobot.TIMEOUT_MS)
        self.bl_motor.config_kD(0, self.d, MyRobot.TIMEOUT_MS)
        self.bl_motor.config_kF(0, self.f, MyRobot.TIMEOUT_MS)

        self.fr_motor.config_kP(0, self.p, MyRobot.TIMEOUT_MS)
        self.fr_motor.config_kI(0, self.i, MyRobot.TIMEOUT_MS)
        self.fr_motor.config_kD(0, self.d, MyRobot.TIMEOUT_MS)
        self.fr_motor.config_kF(0, self.f, MyRobot.TIMEOUT_MS)

        self.br_motor.config_kP(0, self.p, MyRobot.TIMEOUT_MS)
        self.br_motor.config_kI(0, self.i, MyRobot.TIMEOUT_MS)
        self.br_motor.config_kD(0, self.d, MyRobot.TIMEOUT_MS)
        self.br_motor.config_kF(0, self.f, MyRobot.TIMEOUT_MS)


    turn_rate_p = ntproperty('/gyro/turn_rate_p', 0)
    turn_rate_i = ntproperty('/gyro/turn_rate_i', 0)
    turn_rate_d = ntproperty('/gyro/turn_rate_d', 0)

    turn_rate_pid_input_range = ntproperty('/gyro/pid_input_range', 180)
    turn_rate_pid_output_range = ntproperty('/gyro/pid_output_range', 1)

    pause_time = ntproperty('/gyro/pause_time', 1)

    max_turn_rate = ntproperty("/gyro/max_turn_rate", 120)

    def robotInit(self):

        self.BUTTON_RBUMPER = 6
        self.BUTTON_LBUMPER = 5

        self.LY_AXIS = 1
        self.LX_AXIS = 0
        self.RX_AXIS = 4
        self.RY_AXIS = 5

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

        self.deadzone_amount = 0.15

        self.control_state = "speed"

        self.spinman = ctre.wpi_talonsrx.WPI_TalonSRX(5)

        self.littlearms1 = wpilib.Servo(7)
        self.littlearms2 = wpilib.Servo(8)

        self.joystick = wpilib.Joystick(0)

        NetworkTables.addEntryListener(self.entry_listener)

        self.use_pid = False
        self.prev_pid_toggle_btn_value = False

        self.navx = navx.AHRS.create_spi()

        self.timer = wpilib.Timer()

        self.init_time = 0

        self.desired_rate = 0
        self.pid_turn_rate = 0

        def normalized_navx():
            return self.get_normalized_angle(self.navx.getAngle())

        self.angle_pid = wpilib.PIDController(self.turn_rate_p, self.turn_rate_i, self.turn_rate_d, self.navx.getAngle, self.set_pid_turn_rate)
        #self.turn_rate_pid.
        #self.turn_rate_pid.
        self.angle_pid.setInputRange(-self.turn_rate_pid_input_range, self.turn_rate_pid_input_range)
        self.angle_pid.setOutputRange(-self.turn_rate_pid_output_range, self.turn_rate_pid_output_range)
        self.angle_pid.setContinuous(True)

        self.turn_rate_values = [0] * 10


    def set_pid_turn_rate(self, turn_rate):
        self.pid_turn_rate = -turn_rate
        #print('turn_rate:', turn_rate)


    def get_normalized_angle(self, unnormalized_angle):

        angle = unnormalized_angle % 360

        if angle > 180:
            return angle - 360
        elif angle < -180:
            return angle + 360
        else:
            return angle

    def entry_listener(self, key, value, is_new):
        try:
            if key == '/gyro/turn_rate_p':
                self.angle_pid.setP(self.turn_rate_p)
            elif key == '/gyro/turn_rate_i':
                self.angle_pid.setI(self.turn_rate_i)
            elif key == '/gyro/turn_rate_d':
                self.angle_pid.setD(self.turn_rate_d)

            if "encoders" in key:
                self.setEncoderPids()
        except Exception as oopsy:
            print("There was an oopsy: " + str(oopsy))



    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        #self.robot_drive.mecanumDrive_Cartesian(1, 1, 0, 0)
        pass

    def autonomousPeriodic(self):
        pass


    def teleopInit(self):
        self.desired_angle = 0
        self.navx.reset()
        self.angle_pid.disable()

        self.driveStates = {
            'velocity': self.velocity_transitions,
            'position': self.position_transitions,
            'rotation': self.rotation_transitions,
            'leave_special': self.leave_special_transitions
        }

        self.drive_sm = State_Machine(self.driveStates)

        self.drive_sm.set_state('velocity')

    def on_pid_toggle(self):
        """When button 4 is pressed, use_pid is toggled"""
        pid_toggle_btn_value = self.joystick.getRawButton(4)

        if not self.prev_pid_toggle_btn_value and pid_toggle_btn_value:
            self.use_pid = not self.use_pid
            print('PID changed to ' + str(self.use_pid))

        self.prev_pid_toggle_btn_value = pid_toggle_btn_value



    def velocity_transitions(self):
        if self.joystick.getRawButton(self.BUTTON_LBUMPER):
            return 'rotation'
        elif self.joystick.getRawButton(self.BUTTON_RBUMPER):
            return 'position'

        return 'velocity'

    def position_transitions(self):
        if self.joystick.getRawButton(self.BUTTON_RBUMPER):
            return 'position'
        return 'leave_special'

    def rotation_transitions(self):
        if self.joystick.getRawButton(self.BUTTON_LBUMPER):
            return 'rotation'

        return 'leave_special'

    def leave_special_transitions(self):
        if self.deadzone(self.joystick.getRawAxis(self.LX_AXIS)) == 0 and self.deadzone(self.joystick.getRawAxis(self.LY_AXIS) ) ==0 and self.deadzone(self.joystick.getRawAxis(self.RX_AXIS)) == 0 and self.deadzone(self.joystick.getRawAxis(self.RY_AXIS)) == 0:
            return 'velocity'

        return 'leave_special'

    def teleopPeriodic(self):
        self.drive_sm.run()

        #print(f'gyro_angle: {self.navx.getAngle()}')

        js_vertical_2 = self.joystick.getRawAxis(3)
        js_horizontal_2 = self.joystick.getRawAxis(4)
        if self.joystick.getRawButton(5):
            angle = math.atan2(js_horizontal_2, js_vertical_2)

        else:
            # self.desired_rate = self.deadzone(js_horizontal_2) * self.max_turn_rate

            # self.turn_rate_values = self.turn_rate_values[1:] + [self.navx.getRate()]
            # average_values = sum(self.turn_rate_values) / len(self.turn_rate_values)

            #print(f"desired: {self.desired_rate}, measured: {self.navx.getRate()}, average_measured: {average_values}, pid: {self.pid_turn_rate}")

            # self.turn_rate_pid.setSetpoint(self.desired_rate)

            #x_speed = self.deadzone(-self.joystick.getRawAxis(1), .15) * self.max_speed

            # self.on_pid_toggle()
            pass

        if self.control_state == "position" and self.joystick.getRawButton(self.BUTTON_RBUMPER):
            # mode for ongoing position mode
            fl, bl, fr, br = driveCartesian(self.joystick.getRawAxis(self.LX_AXIS), -self.joystick.getRawAxis(self.LY_AXIS), self.joystick.getRawAxis(self.RX_AXIS), self.navx.getAngle())

            self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Position, fl*self.displacement_multiplier)
            self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Position, fr*self.displacement_multiplier)
            self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.Position, bl*self.displacement_multiplier)
            self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.Position, br*self.displacement_multiplier)

        elif self.joystick.getRawButton(self.BUTTON_RBUMPER):
            # code for when position mode first starts
            self.control_state = "position"
            MyRobot.TIMEOUT_MS
            fl, bl, fr, br = driveCartesian(self.joystick.getRawAxis(self.LX_AXIS),
                                            self.joystick.getRawAxis(self.LY_AXIS),
                                            self.joystick.getRawAxis(self.RX_AXIS), self.navx.getAngle())

            self.fl_motor.setQuadraturePosition(int(fl), MyRobot.TIMEOUT_MS)
            self.fr_motor.setQuadraturePosition(int(fr), MyRobot.TIMEOUT_MS)
            self.br_motor.setQuadraturePosition(int(br), MyRobot.TIMEOUT_MS)
            self.bl_motor.setQuadraturePosition(int(bl), MyRobot.TIMEOUT_MS)
            self.navx.reset()

        elif self.control_state == "position":
            # code for when position mode ends
            if abs(self.joystick.getRawAxis(self.LX_AXIS)) < self.deadzone_amount and abs(
                    self.joystick.getRawAxis(self.LY_AXIS)) < self.deadzone_amount and abs(
                    self.joystick.getRawAxis(self.RX_AXIS)) < self.deadzone_amount:
                self.control_state = "speed"

        elif self.joystick.getRawButton(self.BUTTON_LBUMPER) and self.control_state == "speed":
            # code for entering rotation mode
            self.control_state = "rotation"
            print('button is pressed')
            self.navx.reset()
            self.angle_pid.enable()

        elif self.joystick.getRawButton(self.BUTTON_LBUMPER) and self.control_state == "rotation":
            # code for rotation mode
            angle_X = self.joystick.getRawAxis(self.RX_AXIS)
            angle_Y = self.joystick.getRawAxis(self.RY_AXIS)
            hypotenuse = math.hypot(angle_X, angle_Y)
            if hypotenuse > .9:
                angle_rad = math.atan2(angle_Y, angle_X)
                angle_deg = math.degrees(angle_rad)
                print(f'angle_deg: {angle_deg}    pid_ouput: {self.angle_pid.get()}')
                self.angle_pid.setSetpoint(angle_deg)
                fl, bl, fr, br = driveCartesian(0, 0, self.angle_pid.get(), self.navx.getAngle())
                self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fl)
                self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, bl)
                self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fr)
                self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, br)





        elif self.control_state == "rotation":
            # code for exiting rotation mode
            print('out of rotation mode')
            self.angle_pid.disable()
            self.control_state = "speed"

        else:
            # code for speed mode
            x_speed = self.deadzone(self.joystick.getRawAxis(self.LX_AXIS), self.deadzone_amount)
            y_speed = self.deadzone(self.joystick.getRawAxis(self.LY_AXIS), self.deadzone_amount)
            z_speed = self.deadzone(js_horizontal_2)
            fl, bl, fr, br = driveCartesian(x_speed, -y_speed, z_speed, self.navx.getAngle())
            self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fl)
            self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, bl)
            self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fr)
            self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, br)


        # if self.joystick.getRawButton(2):
        #     x_speed = -.4
        #     y_speed = 0
        #     z_speed = 0
        # elif self.joystick.getRawButton(3):
        #     x_speed = .4
        #     y_speed = 0
        #     z_speed = 0
        # else:
        #     x_speed = self.deadzone(-self.joystick.getRawAxis(0), .15)
        #     y_speed = self.deadzone(-self.joystick.getRawAxis(1), .15)
        #     z_speed = self.deadzone(-js_horizontal_2)
        #
        # fl, bl, fr, br = driveCartesian(-x_speed, y_speed, z_speed, self.navx.getAngle())
        #
        # if not self.use_pid:
        #     self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fl)
        #     self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, bl)
        #     self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fr)
        #     self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, br)
        #
        #
        # else:
        #     fl_speed = self.to_motor_speed(fl * self.max_speed, self.ticks_per_rev)
        #     bl_speed = self.to_motor_speed(bl * self.max_speed, self.ticks_per_rev)
        #     fr_speed = self.to_motor_speed(fr * self.max_speed, self.ticks_per_rev)
        #     br_speed = self.to_motor_speed(br * self.max_speed, self.ticks_per_rev)
        #
        #     self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, fl_speed)
        #     self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, bl_speed)
        #     self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, fr_speed)
        #     self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, br_speed)


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


    def deadzone(self, value, min = .2):
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