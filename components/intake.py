import rev
import wpilib
from magicbot import StateMachine, feedback, state, timed_state, tunable

import constants
from common.arduino_light import I2CArduinoLight


class Intake(StateMachine):
    """
    This is a brushed DC motor, we do not have feedback and can't do PID
    control
    """

    # Injections
    is_sim: bool
    arduino_light: I2CArduinoLight

    # Tunable
    intake_velocity = tunable(0.75)
    feed_velocity = tunable(0.75)

    kP = tunable(0.0002)
    kI = tunable(0.0000005)
    kD = tunable(0.0)
    kFF = tunable(0.0)
    kMotorClosedLoopRampRate = tunable(0.0)

    # Local configs
    kInverted = True
    beamWithObject = 1100
    beamNoObject = 275
    kMinOutput = -1
    kMaxOutput = 1

    def setup(self):
        # Beam
        self.beam = wpilib.AnalogInput(constants.AnalogIO.BEAM_SENSOR)

        # Intake motor
        motor_type = (
            rev.CANSparkMax.MotorType.kBrushless
            if self.is_sim
            else rev.CANSparkMax.MotorType.kBrushed
        )
        self.motor = rev.CANSparkMax(constants.CANIds.INTAKE, motor_type)

        # XXX: In concept it's good to reset, in practice it's only needed when
        #      we change an ESC and there's the potential for missed
        #      configuration frames
        # self.motor.restoreFactoryDefaults()
        self.motor.setInverted(self.kInverted)
        self.motor.setSmartCurrentLimit(40)
        self.pid = self.motor.getPIDController()
        self.motor.burnFlash()
        self.object_average = [False, False, False]

    def on_enable(self):
        # Update the tunables
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)

    @feedback
    def has_object(self):
        value = self.beam.getValue() > ((self.beamWithObject - self.beamNoObject) / 2)
        self.object_average.append(value)
        self.object_average = self.object_average[-3:]
        if all(self.object_average):
            return True
        else:
            return False

    def intake(self):
        """Grab a note from the floor, always at full speed"""
        self.pid.setReference(1, rev.CANSparkMax.ControlType.kDutyCycle)

    def feed(self):
        """Feed the shooter, always at full speed"""
        self.pid.setReference(1, rev.CANSparkMax.ControlType.kDutyCycle)

    def outtake(self):
        """Mostly for debugging"""
        self.pid.setReference(-1, rev.CANSparkMax.ControlType.kDutyCycle)

    def disable(self):
        self.pid.setReference(0, rev.CANSparkMax.ControlType.kDutyCycle)

    def jiggle(self):
        self.engage(initial_state="jiggle_start")

    @timed_state(duration=0.2, must_finish=True, next_state="jiggle_out")
    def jiggle_start(self, initial_call):
        if not self.has_object():
            if initial_call:
                self.pid.setReference(0.4, rev.CANSparkMax.ControlType.kDutyCycle)
        else:
            self.next_state_now("jiggle_out")

    @timed_state(first=True, duration=0.2, must_finish=True, next_state="jiggle_intake")
    def jiggle_out(self, initial_call):
        if self.has_object():
            if initial_call:
                self.pid.setReference(-0.4, rev.CANSparkMax.ControlType.kDutyCycle)
        else:
            self.next_state_now("jiggle_intake")

    @timed_state(duration=0.1, must_finish=True, next_state="jiggle_stop")
    def jiggle_intake(self, initial_call):
        if initial_call:
            self.pid.setReference(0.4, rev.CANSparkMax.ControlType.kDutyCycle)
        if self.has_object():
            self.next_state_now("jiggle_stop")

    @timed_state(duration=0.3, must_finish=True, next_state="jiggle_stop")
    def deep(self, initial_call):
        self.pid.setReference(1.0, rev.CANSparkMax.ControlType.kDutyCycle)
        if self.has_object():
            self.next_state_now("jiggle_stop")

    @state(must_finish=True)
    def jiggle_stop(self):
        self.pid.setReference(0, rev.CANSparkMax.ControlType.kDutyCycle)
        self.done()

    # @feedback
    def sensor_value(self):
        return self.beam.getValue()
