import rev
import wpilib
from magicbot import StateMachine, default_state, state, timed_state

import constants

from .lobra import LoBras, LoBrasArm, LoBrasHead

INTAKE_SPEED = 10


# TODO MOVE TO INTAKE
class IntakeBeam:
    def setup(self):
        self.beam = wpilib.AnalogInput(2)

    # Informational methods
    def object_in_sensor(self):
        return self.beam.getValue() < 324

    def execute(self):
        pass


class Intake:
    def setup(self):
        self.input_motor = rev.CANSparkMax(
            constants.CANIds.INTAKE, rev.CANSparkMax.MotorType.kBrushless
        )

    def enable(self):
        pass

    def is_object_intaken(self):
        return False

    def feed_shooter(self):
        pass

    def disable(self):
        pass

    def execute(self):
        pass


class IntakeControl(StateMachine):
    lobra: LoBras
    arm: LoBrasArm
    head: LoBrasHead
    intake: Intake

    def grab(self):
        """This function is called from teleop or autonomous to cause the
        shooter to fire"""
        self.engage()

    @state(first=True)
    def position_head(self):
        """Premier etat, position la tete"""
        self.head.set_angle(100)

        if self.head.is_ready():
            self.next_state_now("position_arm")

    @state
    def position_arm(self):
        self.arm.set_angle(0)
        if self.arm.is_ready():
            self.next_state_now("do_intake")

    @state
    def do_intake(self):
        self.intake.enable()

        if self.intake.is_object_intaken():
            self.next_state_now("halt")

    @default_state
    def halt(self):
        """Always called to stop the motor"""
        self.intake.disable()
