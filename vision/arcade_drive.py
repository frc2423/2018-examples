import math
from wpilib.drive import RobotDriveBase

def arcade_drive(xSpeed: float, zRotation: float, squareInputs: bool = True
) -> None:
    """Arcade drive method for differential drive platform.

    :param xSpeed: The robot's speed along the X axis `[-1.0..1.0]`. Forward is positive
    :param zRotation: The robot's zRotation rate around the Z axis `[-1.0..1.0]`. Clockwise is positive
    :param squareInputs: If set, decreases the sensitivity at low speeds.
    """


    xSpeed = RobotDriveBase.limit(xSpeed)
    xSpeed = RobotDriveBase.applyDeadband(xSpeed, 0)

    zRotation = RobotDriveBase.limit(zRotation)
    zRotation = RobotDriveBase.applyDeadband(zRotation, 0)

    if squareInputs:
        # Square the inputs (while preserving the sign) to increase fine
        # control while permitting full power.
        xSpeed = math.copysign(xSpeed * xSpeed, xSpeed)
        zRotation = math.copysign(zRotation * zRotation, zRotation)

    maxInput = math.copysign(max(abs(xSpeed), abs(zRotation)), xSpeed)

    if xSpeed >= 0.0:
        if zRotation >= 0.0:
            leftMotorSpeed = maxInput
            rightMotorSpeed = xSpeed - zRotation
        else:
            leftMotorSpeed = xSpeed + zRotation
            rightMotorSpeed = maxInput
    else:
        if zRotation >= 0.0:
            leftMotorSpeed = xSpeed + zRotation
            rightMotorSpeed = maxInput
        else:
            leftMotorSpeed = maxInput
            rightMotorSpeed = xSpeed - zRotation

    leftMotorSpeed = RobotDriveBase.limit(leftMotorSpeed)
    rightMotorSpeed = RobotDriveBase.limit(rightMotorSpeed)

    return leftMotorSpeed, rightMotorSpeed