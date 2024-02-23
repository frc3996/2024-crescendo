# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import enum


@enum.unique
class TalonIds(enum.IntEnum):
    # Front Left
    drive_1 = 26
    steer_1 = 25
    cancoder_1 = 11

    # Back Left
    drive_2 = 24
    steer_2 = 23
    cancoder_2 = 12

    # Back Right
    drive_3 = 21
    steer_3 = 22
    cancoder_3 = 13

    # Front Right
    drive_4 = 28
    steer_4 = 27
    cancoder_4 = 14


class OIConstants:
    kDriverControllerPort = 0
