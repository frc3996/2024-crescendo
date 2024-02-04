import math
from dataclasses import field

from magicbot import (StateMachine, default_state, feedback, state,
                      timed_state, tunable)
from magicbot.state_machine import StateRef
from wpimath import controller

from common import arduino_light, tools
from components.field import FieldLayout
from components.intake import Intake
from components.lobra import LoBrasArm, LoBrasHead
from components.pixy import Pixy
from components.shooter import Shooter, ShooterFollower
from components.swervedrive import SwerveDrive


class ActionManualGrab(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    intake: Intake
    arduino_light: arduino_light.I2CArduinoLight
    drivetrain: SwerveDrive
    pixy: Pixy
    auto_intake_kp = tunable(0.015)
    auto_intake_ki = tunable(0)
    auto_intake_kd = tunable(0)
    auto_intake_pid = controller.PIDController(0, 0, 0)
    intake_target_angle = tunable(113)

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def start_intake(self, initial_call):
        if initial_call:
            self.intake.intake()

        if self.intake.has_object():
            self.arduino_light.set_RGB(0, 0, 255)
            self.done()

    def done(self) -> None:
        self.intake.disable()
        self.lobras_head.set_angle(0)
        return super().done()


class ActionGrab(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    intake: Intake
    arduino_light: arduino_light.I2CArduinoLight
    drivetrain: SwerveDrive
    pixy: Pixy
    auto_intake_kp = tunable(0.015)
    auto_intake_ki = tunable(0)
    auto_intake_kd = tunable(0)
    auto_intake_pid = controller.PIDController(0, 0, 0)
    intake_target_angle = tunable(113)

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

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
            self.intake.intake()

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

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

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
            self.shooter.shoot_speaker()

        # Use a normal state
        if self.shooter.is_ready():
            self.next_state("feed_start")

    @timed_state(duration=1.5)
    def feed_start(self, initial_call):
        if initial_call:
            self.intake.feed()

    def done(self) -> None:
        self.shooter.disable()
        self.intake.disable()
        return super().done()


class ActionLowShoot(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    shooter_follower: ShooterFollower
    intake: Intake
    start_shoot_angle = tunable(81)

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def position_arm(self):
        self.lobras_arm.set_angle(0)
        if self.lobras_arm.is_ready(acceptable_error=5):
            self.next_state("position_head")

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(self.start_shoot_angle)

        if self.lobras_head.is_ready(acceptable_error=5):
            self.next_state("prepare_to_fire1")

    @state
    def prepare_to_fire1(self):
        self.shooter.shoot_speaker()
        self.shooter_follower.shoot_speaker()
        self.intake.intake()
        print(self.intake.getVelocity())
        if self.intake.has_object():
            self.next_state("prepare_to_fire2")

    @timed_state(duration=1, next_state="prepare_to_fire")
    def prepare_to_fire2(self):
        self.intake.disable()
        pass

    @state
    def prepare_to_fire(self, initial_call):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        # if initial_call:
        self.shooter.shoot_speaker()
        self.shooter_follower.shoot_speaker()

        # Use a normal state
        if self.shooter.is_ready() and self.shooter_follower.is_ready():
            self.next_state("feed_start")

    @timed_state(duration=1.5)
    def feed_start(self, initial_call):
        if initial_call:
            self.intake.feed()

    def done(self) -> None:
        # self.shooter.disable()
        # self.shooter_follower.disable()
        self.intake.disable()
        return super().done()


class ActionShootAmp(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    shooter_follower: ShooterFollower
    intake: Intake
    arm_angle = tunable(104)
    head_angle = tunable(130)

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def position_arm(self):
        self.lobras_arm.set_angle(self.arm_angle)
        if self.lobras_arm.is_ready(acceptable_error=5):
            self.next_state("position_head")

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(self.head_angle)

        if self.lobras_head.is_ready(acceptable_error=5):
            self.next_state("prepare_to_fire")

    @state
    def prepare_to_fire(self, initial_call):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        if initial_call:
            self.shooter.shoot_amp()
            self.shooter_follower.shoot_amp()

        # Use a normal state
        if self.shooter.is_ready() and self.shooter_follower.is_ready():
            self.next_state("feed_start")

    @timed_state(duration=1.5)
    def feed_start(self, initial_call):
        if initial_call:
            self.intake.feed()

    def done(self) -> None:
        self.shooter.disable()
        self.shooter_follower.disable()
        self.intake.disable()
        return super().done()


class ActionStow(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    shooter_follower: ShooterFollower
    intake: Intake

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def position_all(self):
        """Premier etat, position la tete, et on s'assure que plu rien tourne"""
        self.shooter.disable()
        self.shooter_follower.disable()
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


class ActionLowShootAuto(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    shooter_follower: ShooterFollower
    intake: Intake
    start_shoot_angle = tunable(81)
    drivetrain: SwerveDrive
    field: FieldLayout

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def position_arm(self):
        self.lobras_arm.set_angle(0)
        if self.lobras_arm.is_ready(acceptable_error=5):
            self.next_state("adjust_robot_pose")

    @state
    def adjust_robot_pose(self):
        """
        Adjusts the robot's position and orientation to match the target pose.
        """

        # Assume the robot can directly move and rotate to these differences
        # move_to_position(dx, dy, dz)
        # rotate_to(dyaw)  # Assuming yaw rotation aligns the robot towards the target accurately
        self.next_state("calculate_launch_angle")

    @state
    def calculate_launch_angle(self):
        amp_position = self.field.getSpeakerRelativePosition()
        if amp_position is None:
            return
        distance = math.sqrt(amp_position.x**2 + amp_position.y**2)
        angle = tools.calculate_optimal_launch_angle(distance, amp_position.z, 10)
        if angle:
            self.next_state("prepare_to_fire1")

    @state
    def prepare_to_fire1(self):
        self.shooter.shoot_speaker()
        self.shooter_follower.shoot_speaker()
        self.intake.intake()
        # TODO ADJUST THE HEAD
        print(self.intake.getVelocity())
        if self.intake.has_object():
            self.next_state("prepare_to_fire2")

    @timed_state(duration=1, next_state="prepare_to_fire")
    def prepare_to_fire2(self):
        self.intake.disable()
        pass

    @state
    def prepare_to_fire(self, initial_call):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        # if initial_call:
        self.shooter.shoot_speaker()
        self.shooter_follower.shoot_speaker()

        # Use a normal state
        if self.shooter.is_ready() and self.shooter_follower.is_ready():
            self.next_state("feed_start")

    @timed_state(duration=1.5)
    def feed_start(self, initial_call):
        if initial_call:
            self.intake.feed()

    def done(self) -> None:
        # self.shooter.disable()
        # self.shooter_follower.disable()
        self.intake.disable()
        return super().done()
