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

        # line sensors 0 is far left, 6 is far right
        self.line_sensor0 = wpilib.DigitalInput(4)
        self.line_sensor1 = wpilib.DigitalInput(5)
        self.line_sensor2 = wpilib.DigitalInput(1)
        self.line_sensor3 = wpilib.DigitalInput(6)
        self.line_sensor4 = wpilib.DigitalInput(0)
        self.line_sensor5 = wpilib.DigitalInput(7)
        self.line_sensor6 = wpilib.DigitalInput(3)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.robot_drive.arcadeDrive(0, 0)



    def teleopPeriodic(self):
        """This function is called periodically during operator control."""

        if self.joystick.getRawButton(1):
            speed = self.joystick.getY()

            sensor_list = [self.line_sensor0, self.line_sensor1, self.line_sensor2, self.line_sensor3, self.line_sensor4, self.line_sensor5, self.line_sensor6]

            SEES_LINE = True
            turn_rate  = 0

            sensors_count = 0
            for i in range(0, len(sensor_list)):
                sensor = sensor_list[i]
                if sensor.get() == SEES_LINE:
                    turn_rate += i - 3
                    sensors_count += 1

            if sensors_count == 0:
                turn_rate = 0
            else:
                turn_rate = -turn_rate/(sensors_count*3)

            print('turn_rate: ', turn_rate)
            print('sensor_count: ', sensors_count)

            self.robot_drive.arcadeDrive(turn_rate * speed, speed)
        else:
            self.robot_drive.arcadeDrive(self.joystick.getX(), self.joystick.getY())


        print('sensor value:', self.line_sensor0.get(), self.line_sensor1.get(), self.line_sensor2.get(), self.line_sensor3.get())




if __name__ == "__main__":
    wpilib.run(MyRobot)