import math
from enum import IntEnum

MAX_ANGULAR_VEL = math.degrees(math.pi * 2)
MAX_ANGULAR_ACCEL = math.degrees(math.pi * 5)


class CANIds(IntEnum):
    SWERVE_ROTATE_FL = 25
    SWERVE_DRIVE_FL = 26
    SWERVE_CANCODER_FL = 11

    SWERVE_ROTATE_FR = 27
    SWERVE_DRIVE_FR = 28
    SWERVE_CANCODER_FR = 14

    SWERVE_ROTATE_RL = 23
    SWERVE_DRIVE_RL = 24
    SWERVE_CANCODER_RL = 12

    SWERVE_ROTATE_RR = 22
    SWERVE_DRIVE_RR = 21
    SWERVE_CANCODER_RR = 13

    ARM_LEFT = 8
    ARM_RIGHT = 30
    HEAD_ANGLE_MOTOR = 7

    INTAKE_GRABBER = 10
    SHOOTER_LEFT = 6
    SHOOTER_RIGHT = 20

    CLIMB_RIGHT = 2
    CLIB_LEFT = 5
