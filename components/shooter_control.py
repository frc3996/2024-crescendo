from lobra import LoBras
from magicbot import StateMachine, default_state, state, timed_state

from components import Intake, Shooter


class ShooterControl(StateMachine):
    lobra: LoBras
    shooter: Shooter
    intake: Intake

    def fire(self):
        """This function is called from teleop or autonomous to cause the
        shooter to fire"""
        self.engage()

    @state(first=True)
    def prepare_to_fire(self):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        self.shooter.enable_shooter()

        if self.shooter.is_ready():
            self.next_state_now("firing")

    @timed_state(duration=1, must_finish=True)
    def firing(self):
        """Fires the ball"""
        self.intake.feed_shooter()

    @default_state
    def halt(self):
        """Always called to stop the motor"""
        self.shooter.disable_shooter()
