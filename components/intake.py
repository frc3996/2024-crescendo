import rev
import wpilib
from magicbot import StateMachine, default_state, feedback, state, timed_state

import constants


class Intake:
    beamWithObject = 1100
    beamNoObject = 275

    def setup(self):
        # Beam
        self.beam = wpilib.AnalogInput(0)

        # Intake motor
        self.motor = rev.CANSparkMax(
            constants.CANIds.INTAKE, rev.CANSparkMax.MotorType.kBrushless
        )

        self.motor.restoreFactoryDefaults()
        self.motor.setControlFramePeriodMs(0)  # Control frame from the rio?
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus0, 20)  # Faults and output (default 10ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus3, 500)   # Analog sensor (default 50ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus4, 60000)   # Alternate encoder (default 20ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus5, 60000)   # Absolute encoder Pos/Angle (default 200ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus6, 60000)   # Absolute encoder Vel/Freq (default 200ms)
        self.motor.setInverted(False)

        self.motor.burnFlash()

    @feedback
    def sensor_value(self):
        return self.beam.getValue()

    def enable(self):
        self.motor.set(0.5)

    def disable(self):
        self.motor.set(0)

    def has_object(self):
        if self.beam.getValue() > ((self.beamWithObject - self.beamNoObject) / 2):
            return True
        else:
            return False

    def execute(self):
        pass
