import math
from dataclasses import field

from magicbot import (StateMachine, default_state, feedback, state,
                      timed_state, tunable)
from magicbot.state_machine import StateRef
from wpimath import controller, units

from common import arduino_light, tools
from components.field import FieldLayout
from components.intake import Intake
from components.lobra import LoBrasArm, LoBrasHead
from components.pixy import Pixy
from components.shooter import Shooter, ShooterFollower
from components.swervedrive import SwerveDrive
from components.climber import Climber
from common.path_helper import PathHelper


class ActionDummy(StateMachine):
    x = tunable(0)

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)


    @state(first=True)
    def start(self):
        self.next_state("bob")

    @timed_state(duration=1, next_state="foo")
    def bob(self):
        pass

    @timed_state(duration=1, next_state="finish", must_finish=True)
    def foo(self):
        pass

    @state
    def finish(self, initial_call):
        if initial_call:
            self.x += 1
        pass

    def done(self):
        super().done()


class ActionStow(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True, must_finish=True)
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


class ActionGrabManual(StateMachine):
    lobras_head: LoBrasHead
    intake: Intake
    arduino_light: arduino_light.I2CArduinoLight
    actionStow: ActionStow

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def start_intake(self, initial_call):
        if initial_call:
            self.intake.intake()

        if self.intake.has_object():
            self.arduino_light.set_RGB(255, 136, 0)
            self.next_state("finish")

    def done(self) -> None:
        self.actionStow.engage()
        return super().done()


class ActionOuttake(StateMachine):
    intake: Intake

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def outtake(self, initial_call):
        self.intake.outtake()

    def done(self) -> None:
        self.intake.disable()
        return super().done()


class ActionGrabAuto(StateMachine):
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
    intake_target_angle = tunable(108)
    intake_target_speed = tunable(1.25)
    actionStow: ActionStow

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def position_head(self):
        """Premier etat, position la tete"""
        if self.intake.has_object():
            self.next_state("finish")
            return

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
            fwd = self.intake_target_speed * (1 - res)
            error = self.auto_intake_pid.calculate(offset, 0)
        else:
            fwd = 0.5
            error = 0
        self.drivetrain.relative_rotate(-error)
        self.drivetrain.set_relative_automove_value(-fwd, 0)

        if self.intake.has_object():
            self.next_state("finish")

    @state
    def finish(self):
        self.arduino_light.set_RGB(255, 136, 0)
        self.intake.disable()
        self.lobras_head.set_angle(0)

    def done(self) -> None:
        self.actionStow.engage()
        return super().done()


class ActionShoot(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    actionStow: ActionStow

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
        self.actionStow.engage()
        return super().done()


class ActionLowShoot(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    start_shoot_angle = tunable(81)
    actionStow: ActionStow

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
        self.intake.intake()
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

        # Use a normal state
        if self.shooter.is_ready():
            self.next_state("feed_start")

    @timed_state(duration=1.5)
    def feed_start(self, initial_call):
        if initial_call:
            self.intake.feed()

    def done(self) -> None:
        self.actionStow.engage()
        return super().done()


class ActionShootAmp(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    arm_angle = tunable(104)
    head_angle = tunable(130)
    actionStow: ActionStow

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

        # Use a normal state
        if self.shooter.is_ready():
            self.next_state("feed_start")

    @timed_state(duration=1.5)
    def feed_start(self, initial_call):
        if initial_call:
            self.intake.feed()

    def done(self) -> None:
        self.actionStow.engage()
        return super().done()


class ActionShootAmpAuto(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    arm_angle = tunable(120)
    head_angle = tunable(120)
    drivetrain: SwerveDrive
    path_kp = tunable(2)
    path_ki = tunable(0)
    path_kd = tunable(0)
    path_profile = tunable(2)
    speed_to_wall = tunable(0.75)
    actionStow: ActionStow

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=False)
    def position_arm(self):
        self.lobras_arm.set_angle(self.arm_angle)
        if self.lobras_arm.is_ready(acceptable_error=5):
            self.next_state("position_head")

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(self.head_angle)

        if self.lobras_head.is_ready(acceptable_error=5):
            self.next_state("go_to_place")

    @state(first=True)
    def go_to_place(self, initial_call):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        if initial_call:
            self.auto_path = PathHelper(self.drivetrain, "amp_shoot", kp=self.path_kp, ki=self.path_ki, kd=self.path_kd, profile_kp=self.path_profile)
            self.auto_path.init_path()

        if self.auto_path.distance_to_end() < 2:
            self.lobras_arm.set_angle(self.arm_angle)
            self.lobras_head.set_angle(self.head_angle)
        if self.auto_path.distance_to_end() < 0.5:
            self.shooter.shoot_amp()

        self.auto_path.move_to_end()

        if not self.lobras_arm.is_ready(acceptable_error=5):
            return
        if not self.lobras_head.is_ready(acceptable_error=5):
            return
        if not self.auto_path.robot_reached_end_position():
            return

        self.next_state("prepare_to_fire")

    @timed_state(duration=0.25, next_state="feed_start")
    def prepare_to_fire(self, initial_call):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        ##if initial_call:
          ##  self.shooter.shoot_amp()
        self.drivetrain.set_absolute_automove_value(0, self.speed_to_wall)
        # Use a normal state
        # if self.shooter.is_ready():
        #     self.next_state("feed_start")

    @timed_state(duration=0.5)
    def feed_start(self, initial_call):
        self.intake.feed()

    def done(self) -> None:
        self.actionStow.engage()
        return super().done()


class FeedAndRetract(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    actionStow: ActionStow

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @timed_state(first=True, duration=0.5, must_finish=True, next_state="stop_and_retract")
    def feed(self):
        self.intake.feed()

    def done(self) -> None:
        self.actionStow.engage()
        return super().done()


class ActionShootAmpAssisted(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    arm_angle = tunable(120)
    head_angle = tunable(120)
    drivetrain: SwerveDrive
    feedAndRetract: FeedAndRetract
    ready_to_fire = False
    arduino_light: arduino_light.I2CArduinoLight
    actionStow: ActionStow

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def rotate(self, initial_call):
        self.ready_to_fire = False

        if initial_call:
            self.auto_path = PathHelper(self.drivetrain, "amp_shoot")
            self.auto_path.init_path()

        self.auto_path.target_end_angle()

        if self.drivetrain.angle_reached():
            self.next_state("prepare_to_shoot")

    @state
    def prepare_to_shoot(self):
        self.lobras_head.set_angle(self.head_angle)
        self.lobras_arm.set_angle(self.arm_angle)
        self.shooter.shoot_amp()

        if not self.lobras_arm.is_ready(acceptable_error=5):
            return
        if not self.lobras_head.is_ready(acceptable_error=5):
            return
        self.next_state("wait_release")

    @state
    def wait_release(self):
        self.ready_to_fire = True
        self.arduino_light.set_RGB(255, 0, 0)

    def done(self) -> None:
        if self.ready_to_fire:
            self.ready_to_fire = False
            self.feedAndRetract.engage()
        else:
            self.actionStow.engage()
        return super().done()


class ActionLowShootAuto(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    start_shoot_angle = tunable(81)
    drivetrain: SwerveDrive
    field_layout: FieldLayout
    is_sim: bool
    arduino_light: arduino_light.I2CArduinoLight
    actionStow: ActionStow

    Z_OFFSET = tunable(0.3)
    X_OFFSET = tunable(0.43)
    THROW_VELOCITY = tunable(10.8)
    THROW_OFFSET = tunable(45)
    ARM_OFFSET = tunable(0)

    @feedback
    def get_best_angle(self):
        speaker_position = self.field_layout.getSpeakerRelativePosition()
        if speaker_position is None:
            return
        distance = math.sqrt(speaker_position.x**2 + speaker_position.y**2) + self.X_OFFSET
        height_difference = speaker_position.z - self.Z_OFFSET
        angle = tools.calculate_optimal_launch_angle(distance, height_difference, self.THROW_VELOCITY)
        height = speaker_position.z

        rough_angle = math.degrees(math.atan(height_difference/distance))
        return [distance, height, angle, rough_angle]

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def position_arm(self):
        if self.intake.has_object() is False:
            self.next_state("finish")
            return

        self.lobras_arm.set_angle(self.ARM_OFFSET)
        self.lobras_head.set_angle(self.THROW_OFFSET)
        if self.lobras_arm.is_ready(acceptable_error=3) or self.is_sim:
            self.next_state("prepare_to_fire")

    # @state
    def set_launch_rotation(self):
        speaker_position = self.field_layout.getSpeakerRelativePosition()
        if speaker_position is None:
            return

        # Rotate the robot
        target_angle = tools.compute_angle(speaker_position.X(), speaker_position.Y())
        self.drivetrain.set_angle(target_angle + 180) # We throw from behind
        if self.drivetrain.angle_reached(acceptable_error=1):
            self._launch_rotation = target_angle
            return True
            # self.next_state("set_launch_angle")
        return False

    # @state
    def set_launch_angle(self):
        speaker_position = self.field_layout.getSpeakerRelativePosition()
        if speaker_position is None:
            return

        # target_angle = tools.compute_angle(speaker_position.X(), speaker_position.Y())
        # if abs(self._launch_rotation - target_angle) > 2:
        #     self.next_state_now("set_launch_rotation")

        # Set the head angle
        distance = math.sqrt(speaker_position.x**2 + speaker_position.y**2)
        angle = tools.calculate_optimal_launch_angle(distance + self.X_OFFSET, speaker_position.z - self.Z_OFFSET, self.THROW_VELOCITY)
        self.lobras_head.set_angle(angle + self.THROW_OFFSET)
        if self.lobras_head.is_ready(acceptable_error=1):
            return True
            # self.next_state("prepare_to_fire")
        return False

    @timed_state(duration=3, next_state="fire")
    def prepare_to_fire(self):
        res1 = self.set_launch_rotation()
        res2 = self.set_launch_angle()
        self.shooter.shoot_speaker()
        if self.shooter.is_ready() and res1 and res2:
            self.next_state_now("fire")

    @timed_state(must_finish=True, duration=0.5, next_state="finish")
    def fire(self):
        self.intake.feed()

    @state
    def finish(self):
        self.intake.disable()
        self.shooter.disable()
        self.lobras_head.set_angle(0)
        self.arduino_light.set_RGB(0, 255, 0)
        pass

    def done(self) -> None:
        self.actionStow.engage()
        return super().done()


class ActionHighShootAuto(ActionLowShootAuto):
    Z_OFFSET = tunable(1.03)
    X_OFFSET = tunable(-0.4)
    THROW_VELOCITY = tunable(10.8)
    ARM_OFFSET = tunable(100)
    THROW_OFFSET = tunable(140)


class ActionDewinch(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    climber: Climber

    arm_angle = tunable(123)
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
            self.next_state("dewinch")

    @state
    def dewinch(self, initial_call):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        self.climber.dewinch()

    def done(self) -> None:
        return super().done()


class ActionWinch(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    climber: Climber
    shooter: Shooter
    intake: Intake
    field_layout: FieldLayout
    drivetrain: SwerveDrive

    ARM_ANGLE = tunable(123)
    HEAD_ANGLE = tunable(130)

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def set_rotation(self):
        target_position = self.field_layout.getTagRelativePosition(15)
        if target_position is None:
            return

        # Rotate the robot
        target_rotation = tools.compute_angle(target_position.X(), target_position.Y())
        self.drivetrain.set_angle(target_rotation) # We throw from behind
        if self.drivetrain.angle_reached(acceptable_error=1):
            self.next_state("position_arm")

    @state
    def position_arm(self):
        self.lobras_arm.set_angle(self.ARM_ANGLE)
        if self.lobras_arm.is_ready(acceptable_error=5):
            self.next_state("position_head")

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(self.HEAD_ANGLE)

        if self.lobras_head.is_ready(acceptable_error=5):
            self.next_state("dewinch")


    @timed_state(duration=2, next_state="move_forward")
    def dewinch(self):
        self.climber.dewinch()

    @state
    def move_forward(self):
        self.next_state("winch")

    @timed_state(duration=2)
    def winch(self):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        self.climber.winch()
        if self.climber.climber_in_closed_position():
            self.next_state("shoot")

    @state
    def shoot(self):
        self.shooter.shoot_amp()
        if self.intake.has_object():
            self.next_state("finish")

    @state
    def finish(self):
        pass

    def done(self) -> None:
        return super().done()
