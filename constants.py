import math
from enum import IntEnum

# Swerve
MAX_ANGULAR_VEL = math.degrees(math.pi * 2) * 2
MAX_ANGULAR_ACCEL = math.degrees(math.pi * 5) * 2
LOWER_INPUT_THRESH = 0.1  # Minimum speed before considered 0
# MAX_WHEEL_SPEED = 5 / 10 # 4  # meter per second
# MAX_MODULE_SPEED = 5.625 / 10 # 4.5

SWERVE_DIRECTION_GEAR_RATIO = (14 / 50) * (10 / 60)

class AnalogIO(IntEnum):
    BEAM_SENSOR = 0
    PIXY_OFFSET = 1

class DigitalIO(IntEnum):
    CLIMBER_LIMIT_SWITCH = 1
    PIXY_VALID = 0


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

    ARM_LEFT = 41
    ARM_RIGHT = 42
    HEAD_ANGLE_MOTOR = 43

    INTAKE = 54
    SHOOTER_LEFT = 52
    SHOOTER_RIGHT = 53

    CLIMB_RIGHT = 60
    CLIMB_LEFT = 61
