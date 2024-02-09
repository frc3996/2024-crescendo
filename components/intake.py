import rev
import wpilib
from magicbot import (StateMachine, default_state, feedback, state,
                      timed_state, tunable)

import constants


class Intake:
    beamWithObject = 1100
    beamNoObject = 275
    intake_velocity = tunable(4000)
    feed_velocity = tunable(16000)

    kP = tunable(0.0002)
    kI = tunable(0.0000005)
    kD = tunable(0.0)
    kFF = tunable(0.0)
    kMotorClosedLoopRampRate = tunable(0.0)
    kInverted = False

    kMinOutput = -1
    kMaxOutput = 1

    def setup(self):
        # Beam
        self.beam = wpilib.AnalogInput(0)

        # Intake motor
        self.motor = rev.CANSparkMax(
            constants.CANIds.INTAKE, rev.CANSparkMax.MotorType.kBrushless
        )

        # self.motor.restoreFactoryDefaults()
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
        self.pid = self.motor.getPIDController()
        self.pid.setFeedbackDevice(self.encoder)
        self.__target_velocity = 0

        self.motor.burnFlash()

    def on_enable(self):
        # Update the tunables
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)
        self.pid.setP(self.kP)
        self.pid.setI(self.kI)
        self.pid.setD(self.kD)
        self.pid.setFF(self.kFF)

    # @feedback
    def sensor_value(self):
        return self.beam.getValue()

    def __set_velocity(self, velocity):
        self.__target_velocity = velocity
        self.pid.setReference(velocity, rev.CANSparkMax.ControlType.kVelocity)

    def intake(self):
        self.__set_velocity(self.intake_velocity)

    def feed(self):
        self.__set_velocity(self.feed_velocity)

    @feedback
    def getVelocity(self):
        return self.encoder.getVelocity()

    def is_ready(self):
        return abs(self.getVelocity() - self.__target_velocity) < 25

    def disable(self):
        self.__target_velocity = 0
        self.motor.set(0)

    # @feedback
    def has_object(self):
        if self.beam.getValue() < ((self.beamWithObject - self.beamNoObject) / 2):
            return True
        else:
            return False

    def execute(self):
        self.getVelocity()
