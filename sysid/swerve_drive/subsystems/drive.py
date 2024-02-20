# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger
from phoenix6.configs import (FeedbackConfigs, MotorOutputConfigs,
                              Slot0Configs, config_groups)
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import PositionDutyCycle, VelocityVoltage, VoltageOut
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import FeedbackSensorSourceValue, NeutralModeValue
from wpilib import sysid
from wpimath.units import volts

from constants import TalonIds

DRIVE_GEAR_RATIO = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)
STEER_GEAR_RATIO = (14 / 50) * (10 / 60)
WHEEL_CIRCUMFERENCE = 4 * 2.54 / 100 * math.pi
DRIVE_MOTOR_REV_TO_METRES = WHEEL_CIRCUMFERENCE * DRIVE_GEAR_RATIO
STEER_MOTOR_REV_TO_RAD = math.tau * STEER_GEAR_RATIO


def steer_config(steer: TalonFX, steer_cancoder, steer_reversed, rotor_offset):
    # Configure steer motor
    steer_config = steer.configurator

    steer_motor_config = MotorOutputConfigs()
    steer_motor_config.neutral_mode = NeutralModeValue.BRAKE
    steer_motor_config.inverted = (
        config_groups.InvertedValue.CLOCKWISE_POSITIVE
        if steer_reversed
        else config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )

    steer_feedback = FeedbackConfigs()
    steer_feedback.with_sensor_to_mechanism_ratio(1 / STEER_GEAR_RATIO)
    steer_feedback.feedback_remote_sensor_id = steer_cancoder
    steer_feedback.feedback_sensor_source = FeedbackSensorSourceValue.REMOTE_CANCODER
    steer_feedback.with_feedback_rotor_offset(rotor_offset)

    # configuration for motor pid
    steer_pid = Slot0Configs().with_k_p(3).with_k_i(0).with_k_d(0.1)

    steer_config.apply(steer_motor_config)
    steer_config.apply(steer_pid, 0.01)
    steer_config.apply(steer_feedback)


def drive_config(drive: TalonFX, drive_reversed):
    # Configure drive motor
    drive_config = drive.configurator

    drive_motor_config = MotorOutputConfigs()
    drive_motor_config.neutral_mode = NeutralModeValue.BRAKE
    drive_motor_config.inverted = (
        config_groups.InvertedValue.CLOCKWISE_POSITIVE
        if drive_reversed
        else config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )

    drive_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
        1 / DRIVE_MOTOR_REV_TO_METRES
    )

    # configuration for motor pid and feedforward
    drive_pid = Slot0Configs().with_k_p(1.0868).with_k_i(0).with_k_d(0)

    drive_config.apply(drive_motor_config)
    drive_config.apply(drive_pid, 0.01)
    drive_config.apply(drive_gear_ratio_config)


class Drive(Subsystem):
    def __init__(self) -> None:
        # The motors on the left side of the drive
        self.drive_1 = TalonFX(TalonIds.drive_1)
        self.drive_2 = TalonFX(TalonIds.drive_2)
        self.drive_3 = TalonFX(TalonIds.drive_3)
        self.drive_4 = TalonFX(TalonIds.drive_4)
        self.drive_motors = [self.drive_1, self.drive_2, self.drive_3, self.drive_4]
        for drive_motor in self.drive_motors:
            # Reduce CAN status frame rates before configuring
            drive_motor.get_fault_field().set_update_frequency(
                frequency_hz=4, timeout_seconds=0.01
            )
            drive_config(drive_motor, False)

        self.steer_1 = TalonFX(TalonIds.steer_1)
        self.steer_2 = TalonFX(TalonIds.steer_3)
        self.steer_3 = TalonFX(TalonIds.steer_2)
        self.steer_4 = TalonFX(TalonIds.steer_4)

        self.steer_motors = [self.steer_1, self.steer_2, self.steer_3, self.steer_4]
        self.cancoders = [
            (TalonIds.cancoder_1, 0),
            (TalonIds.cancoder_2, 0),
            (TalonIds.cancoder_3, 0),
            (TalonIds.cancoder_4, 0),
        ]
        for steer_motor, steer_cancoder in zip(self.steer_motors, self.cancoders):
            # Reduce CAN status frame rates before configuring
            steer_motor.get_fault_field().set_update_frequency(
                frequency_hz=4, timeout_seconds=0.01
            )
            steer_config(steer_motor, steer_cancoder[0], False, steer_cancoder[1])

        # Tell SysId how to plumb the driving voltage to the motors.
        def drive(voltage: volts) -> None:
            voltage_request = VoltageOut(voltage)
            for drive_motor in self.drive_motors:
                drive_motor.set_control(voltage_request)

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(recordState=self.recordState),
            SysIdRoutine.Mechanism(drive, self.log, self),
        )

        self.logger_inited = False

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        # Record a frame for the left motors.  Since these share an encoder, we consider
        # the entire group to be one motor.
        for drive_motor, index in zip(self.drive_motors, range(1, 5)):
            sys_id_routine.motor(f"drive-{index}").voltage(
                drive_motor.get_motor_voltage().value
            ).position(drive_motor.get_position().value * WHEEL_CIRCUMFERENCE).velocity(
                drive_motor.get_velocity().value * WHEEL_CIRCUMFERENCE
            )

    def recordState(self, state: sysid.State) -> None:
        if not self.logger_inited:
            SignalLogger.start()
            self.logger_inited = True

        SignalLogger.write_string(
            f"sysid-test-state-{self.getName()}",
            sysid.SysIdRoutineLog.stateEnumToString(state),
        )
        self.sys_id_routine.recordState(state)

    def defaultCommand(self) -> Command:
        return self.run(lambda: None)

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)
