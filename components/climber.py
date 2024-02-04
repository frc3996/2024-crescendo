import rev
import wpilib
from magicbot import (StateMachine, default_state, feedback, state, will_reset_to, tunable,
                      timed_state, tunable)

import constants


class Climber:
    kMotorClosedLoopRampRate = tunable(0.5)
    kInverted = False

    kMinOutput = -1
    kMaxOutput = 1

    target_speed = will_reset_to(0)

    def setup(self):
        # Beam

        # Intake motor
        self.motor = rev.CANSparkMax(
            constants.CANIds.CLIMB_RIGHT, rev.CANSparkMax.MotorType.kBrushless
        )
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)


        self.motor.restoreFactoryDefaults()
        self.motor.setControlFramePeriodMs(0)  # Control frame from the rio?
        self.motor.setPeriodicFramePeriod(
            self.motor.PeriodicFrame.kStatus0, 20
        )  # Faults and output (default 10ms)
        self.motor.setPeriodicFramePeriod(
            self.motor.PeriodicFrame.kStatus3, 500
        )  # Analog sensor (default 50ms)
        self.motor.setPeriodicFramePeriod(
            self.motor.PeriodicFrame.kStatus4, 60000
        )  # Alternate encoder (default 20ms)
        self.motor.setPeriodicFramePeriod(
            self.motor.PeriodicFrame.kStatus5, 60000
        )  # Absolute encoder Pos/Angle (default 200ms)
        self.motor.setPeriodicFramePeriod(
            self.motor.PeriodicFrame.kStatus6, 60000
        )  # Absolute encoder Vel/Freq (default 200ms)
        self.motor.setInverted(self.kInverted)
        self.encoder = self.motor.getEncoder()


        self.motor.burnFlash()

    def winch(self):
        self.target_speed = 1

    def dewinch(self):
        self.target_speed = -1

    def execute(self):
        self.motor.set(self.target_speed)


class ClimberFollower:
    climber: Climber

    kMotorClosedLoopRampRate = tunable(0.5)
    kInverted = False

    kMinOutput = -1
    kMaxOutput = 1

    target_speed = will_reset_to(0)

    def setup(self):
        # Beam

        # Intake motor
        self.motor = rev.CANSparkMax(
            constants.CANIds.CLIMB_LEFT, rev.CANSparkMax.MotorType.kBrushless
        )
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)


        self.motor.restoreFactoryDefaults()
        self.motor.setControlFramePeriodMs(0)  # Control frame from the rio?
        self.motor.setPeriodicFramePeriod(
            self.motor.PeriodicFrame.kStatus0, 20
        )  # Faults and output (default 10ms)
        self.motor.setPeriodicFramePeriod(
            self.motor.PeriodicFrame.kStatus3, 500
        )  # Analog sensor (default 50ms)
        self.motor.setPeriodicFramePeriod(
            self.motor.PeriodicFrame.kStatus4, 60000
        )  # Alternate encoder (default 20ms)
        self.motor.setPeriodicFramePeriod(
            self.motor.PeriodicFrame.kStatus5, 60000
        )  # Absolute encoder Pos/Angle (default 200ms)
        self.motor.setPeriodicFramePeriod(
            self.motor.PeriodicFrame.kStatus6, 60000
        )  # Absolute encoder Vel/Freq (default 200ms)
        self.motor.setInverted(self.kInverted)
        self.encoder = self.motor.getEncoder()


        self.motor.burnFlash()
        self.motor.follow(self.climber.motor, invert=self.kInverted)

    def execute(self):
        pass
