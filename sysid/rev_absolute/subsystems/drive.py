# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib import sysid

from phoenix6 import SignalLogger
from phoenix6.hardware import TalonFX
from phoenix6.configs import FeedbackConfigs, MotorOutputConfigs
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import VoltageOut

from wpimath.units import volts

from constants import TalonIds


class Drive(Subsystem):
    DRIVE_GEAR_RATIO = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)
    WHEEL_CIRCUMFERENCE = 4 * 2.54 / 100 * math.pi

    def __init__(self) -> None:
        # The motors on the left side of the drive
        self.drive_1 = TalonFX(TalonIds.drive_1)
        self.drive_2 = TalonFX(TalonIds.drive_2)
        self.drive_3 = TalonFX(TalonIds.drive_3)
        self.drive_4 = TalonFX(TalonIds.drive_4)
        self.drive_motors = [self.drive_1, self.drive_2, self.drive_3, self.drive_4]
        for drive_motor in self.drive_motors:
            drive_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
                1 / self.DRIVE_GEAR_RATIO
            )
            drive_config = drive_motor.configurator
            drive_config.apply(drive_gear_ratio_config)

        self.steer_1 = TalonFX(TalonIds.steer_1)
        self.steer_2 = TalonFX(TalonIds.steer_3)
        self.steer_3 = TalonFX(TalonIds.steer_2)
        self.steer_4 = TalonFX(TalonIds.steer_4)

        self.steer_motors = [self.steer_1, self.steer_2, self.steer_3, self.steer_4]
        for steer_motor in self.steer_motors:
            steer_motor_config = MotorOutputConfigs()
            steer_motor_config.neutral_mode = NeutralModeValue.BRAKE
            steer_config = steer_motor.configurator
            steer_config.apply(steer_motor_config)

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
            ).position(
                drive_motor.get_position().value * self.WHEEL_CIRCUMFERENCE
            ).velocity(
                drive_motor.get_velocity().value * self.WHEEL_CIRCUMFERENCE
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
