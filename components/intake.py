import rev
import wpilib
from magicbot import StateMachine, default_state, feedback, state, timed_state

import constants

from .lobra import LoBras, LoBrasArm, LoBrasHead

INTAKE_SPEED = 10


# TODO MOVE TO INTAKE
# Values are 275 with the donut
# 1100 without the donut
class IntakeBeam:
    def setup(self):
        self.beam = wpilib.AnalogInput(0)

    # Informational methods
    def object_in_sensor(self):
        return self.beam.getValue() < 324

    @feedback
    def sensor_value(self):
        return self.beam.getValue()

    def execute(self):
        pass


class Intake:
    def setup(self):
        self.motor = rev.CANSparkMax(
            constants.CANIds.INTAKE, rev.CANSparkMax.MotorType.kBrushless
        )

        self.motor.restoreFactoryDefaults()
        self.motor.setInverted(False)

        self.motor.burnFlash()

    def enable(self):
        self.motor.set(0.5)

    def is_object_intaken(self):
        return False

    def feed_shooter(self):
        pass

    def disable(self):
        self.motor.set(0)

    def execute(self):
        return
        # Update the tunables
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)
        self.pid.setP(self.kP)
        self.pid.setI(self.kI)
        self.pid.setD(self.kD)
        self.pid.setFF(self.kFF)


class IntakeFeed(StateMachine):
    intake: Intake

    def enable(self):
        """This function is called from teleop or autonomous to cause the
        shooter to fire"""
        self.engage()


class IntakeControl(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    intake: Intake

    # Grab
    def grab(self):
        """This function is called from teleop or autonomous to cause the
        shooter to fire"""
        self.engage(self.position_head)

    @state(first=True)
    def invalid_state(self):
        pass

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(110)

        if self.lobras_head.is_ready():
            self.next_state_now("position_arm")

    @state
    def position_arm(self):
        self.lobras_arm.set_angle(0)
        if self.lobras_arm.is_ready():
            self.next_state_now("do_intake")

    @state
    def do_intake(self):
        self.intake.enable()
        self.next_state_now("wait_intake")

        # if self.intake.is_object_intaken():
        #     self.next_state_now("stop")

    @timed_state(duration=4, next_state="stop")
    def wait_intake(self):
        pass

    # Feed
    def feed(self):
        """Premier etat, position la tete"""
        self.engage(self.feed_start)

    @state
    def feed_start(self):
        self.intake.enable()
        self.next_state_now("feed_wait")

    @timed_state(duration=5, next_state="stop")
    def feed_wait(self):
        pass

    # STOP
    @state
    def stop(self):
        """Always called to stop the motor"""
        self.intake.disable()

    def done(self) -> None:
        self.intake.disable()
        return super().done()
