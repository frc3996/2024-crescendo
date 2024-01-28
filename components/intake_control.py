from lobra import LoBras, LoBrasArm, LoBrasHead
from magicbot import StateMachine, default_state, state, timed_state

from components import Intake, Shooter


class IntakeControl(StateMachine):
    lobra: LoBras
    arm: LoBrasArm
    head: LoBrasHead
    shooter: Shooter
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
