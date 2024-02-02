import ntcore
import wpilib
import math
from magicbot import StateMachine, default_state, feedback, state, timed_state, tunable
from wpimath import controller, geometry

from common import arduino_light, limelight, path_helper, tools
from components import *


class ActionGrab(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    intake: Intake
    arduino_light: arduino_light.I2CArduinoLight
    drivetrain: swervedrive.SwerveDrive
    pixy: Pixy
    auto_intake_kp = tunable(0.015)
    auto_intake_ki = tunable(0)
    auto_intake_kd = tunable(0)
    auto_intake_pid = controller.PIDController(0, 0, 0)
    intake_target_angle = tunable(113)

    @state(first=True)
    def position_head(self):
        """Premier etat, position la tete"""
        self.auto_intake_pid.setPID(
            self.auto_intake_kp,
            self.auto_intake_ki,
            self.auto_intake_kd,
        )
        self.auto_intake_pid.reset()

        self.lobras_head.set_angle(self.intake_target_angle)

        if self.lobras_head.is_ready(acceptable_error=10):
            self.next_state("position_arm")

    @state
    def position_arm(self):
        self.lobras_arm.set_angle(0)
        if self.lobras_arm.is_ready(acceptable_error=10):
            self.next_state("start_intake")

    @state
    def start_intake(self, initial_call):
        if initial_call:
            self.intake.enable()

        # # Automove to target
        if self.pixy.get_target_valid():
            offset = self.pixy.get_offset()
            res = tools.map_value(abs(offset), 0, 1000, 0, 1)
            fwd = 0.5 * (1 - res)
            error = self.auto_intake_pid.calculate(offset, 0)
        else:
            fwd = 0.5
            error = 0
        self.drivetrain.relative_rotate(-error)
        self.drivetrain.set_relative_automove_value(-fwd, 0)

        if self.intake.has_object():
            self.arduino_light.set_RGB(0, 0, 255)
            self.next_state("finish")

    # @timed_state(duration=0.05, next_state="finish")
    # def finish_intake(self):
    #     pass

    @state
    def finish(self):
        self.done()

    def done(self) -> None:
        self.intake.disable()
        self.lobras_head.set_angle(0)
        return super().done()


class ActionShoot(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake

    @state(first=True)
    def position_arm(self):
        self.lobras_arm.set_angle(100)
        if self.lobras_arm.is_ready(acceptable_error=90):
            self.next_state("position_head")

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(175)

        if self.lobras_head.is_ready(acceptable_error=10):
            self.next_state("prepare_to_fire")

    @state
    def prepare_to_fire(self, initial_call):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        if initial_call:
            self.shooter.enable()

        # Use a normal state
        if self.shooter.is_ready():
            print("READY")
            self.next_state("feed_start")

    @timed_state(duration=1.5, next_state="finish")
    def feed_start(self, initial_call):
        if initial_call:
            self.intake.enable()

    @state
    def finish(self):
        self.done()


    def done(self) -> None:
        self.shooter.disable()
        self.intake.disable()
        return super().done()


class ActionStow(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake


    @state(first=True)
    def position_all(self):
        """Premier etat, position la tete, et on s'assure que plu rien tourne"""
        self.shooter.disable()
        self.intake.disable()

        # Sets the head angle to 0
        self.lobras_head.set_angle(0)

        # To avoid collision we make sure the head is retracted

        # If the head is extended more than 90 degrees, wait for
        # it and start moving then arm half-way
        if self.lobras_head.get_angle() < 90:
            self.lobras_arm.set_angle(0)
        # Only move the arm half-way if the current angle was already > 30
        elif self.lobras_arm.get_angle() > 30:
                self.lobras_arm.set_angle(30)

        if self.lobras_arm.is_ready() and self.lobras_head.is_ready():
            self.done()
