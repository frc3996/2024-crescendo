# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import rev
from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib import sysid
from wpimath.units import volts


class Shooter(Subsystem):
    def __init__(self) -> None:
        self.motor = rev.CANSparkMax(52, rev.CANSparkMax.MotorType.kBrushless)
        self.motor.restoreFactoryDefaults()
        self._voltage = 0

        # An encoder set up to measure flywheel velocity in radians per second.
        # self.encoder = wpilib.Encoder(kEncoderAChannel, kEncoderBChannel)
        self.encoder = self.motor.getEncoder()

        # Tell SysId how to plumb the driving voltage to the motors.
        def shoot(voltage: volts) -> None:
            self.motor.setVoltage(voltage)
            self._voltage = voltage

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(shoot, self.log, self),
        )

    # def recordState(self, state: sysid.State) -> None:
    #     print("Saving state!!!!!!!!!!!!!!!!!!!!!!!")
    #     self.sys_id_routine.recordState(state)

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        # Record a frame for the left motors.  Since these share an encoder, we consider
        # the entire group to be one motor.
        sys_id_routine.motor("shooter").voltage(self._voltage).velocity(
            self.encoder.getVelocity()
        ).position(self.encoder.getPosition())

    def defaultCommand(self) -> Command:
        return self.run(lambda: None)

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)
