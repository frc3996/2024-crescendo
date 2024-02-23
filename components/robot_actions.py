import math

from magicbot import StateMachine, feedback, state, timed_state, tunable
from magicbot.state_machine import StateRef
from wpimath import controller, geometry

from common import arduino_light, tools
from components.climber import Climber
from components.field import FieldLayout
from components.intake import Intake
from components.lobra import LoBrasArm, LoBrasHead
from components.pixy import Pixy
from components.shooter import Shooter
from components.swervedrive import SwerveDrive
from common.path_helper import PathHelper


class ActionStow(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    is_sim: bool

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True, must_finish=True)
    def position_all(self):
        """Premier etat, position la tete, et on s'assure que plu rien tourne"""
        if self.is_sim:
            self.done()

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

    def done(self):
        super().done()


class ActionDummy(StateMachine):
    x = tunable(0)
    actionStow: ActionStow

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
            print(self.x)
        pass

    def done(self):
        self.actionStow.engage()
        super().done()


class ActionPathTester(StateMachine):
    actionStow: ActionStow
    drivetrain: SwerveDrive
    path_kp = tunable(2)
    path_ki = tunable(0)
    path_kd = tunable(0)
    path_profile = tunable(2)
    force_reset_pose = tunable(False)

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def init(self):
        self.auto_path = PathHelper(
            self.drivetrain,
            "amp_to_1",
            kp=self.path_kp,
            ki=self.path_ki,
            kd=self.path_kd,
            profile_kp=self.path_profile,
        )
        self.auto_path.init_path(force_robot_starting_position=self.force_reset_pose)
        self.next_state("move")

    @state
    def move(self):
        self.auto_path.move_to_end()
        if self.auto_path.robot_reached_end_position():
            print("Reached end!")

    def done(self):
        self.actionStow.engage()
        super().done()


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
    def outtake(self):
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
    auto_intake_kp = tunable(0.001)  # For relative_rotate: 0.015
    auto_intake_ki = tunable(0)  # For relative_rotate: 0
    auto_intake_kd = tunable(0)  # For relative_rotate: 0
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

        if self.lobras_head.is_ready(acceptable_error=5):
            self.next_state("position_arm")

    @state
    def position_arm(self):
        self.lobras_arm.set_angle(0)
        if self.lobras_arm.is_ready(acceptable_error=5):
            self.next_state("start_intake")

    @state
    def start_intake(self, initial_call):
        if self.intake.has_object():
            self.next_state("finish_intaking")

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
        self.drivetrain.set_robot_relative_automove_value(-fwd, -error)

    @state(must_finish=True)
    def finish_intaking(self, initial_call):
        self.lobras_head.set_angle(70)
        self.intake.jiggle()
        if self.intake.is_executing and not initial_call:
            self.next_state("finish")

    @state(must_finish=True)
    def finish(self, initial_call):
        if initial_call:
            self.arduino_light.set_RGB(255, 136, 0)
            self.lobras_head.set_angle(0)
        self.done()

    def done(self) -> None:
        self.actionStow.engage()
        return super().done()


class ActionGrabAssisted(ActionGrabAuto):

    @state
    def finish(self, initial_call):
        if initial_call:
            self.arduino_light.set_RGB(255, 136, 0)
            self.lobras_head.set_angle(0)


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
        if self.lobras_arm.is_ready(acceptable_error=2):
            self.next_state("position_head")

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(175)

        if self.lobras_head.is_ready(acceptable_error=2):
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
    start_shoot_angle = tunable(79)
    actionStow: ActionStow

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def position_arm(self):
        self.lobras_arm.set_angle(0)
        if self.lobras_arm.is_ready(acceptable_error=2):
            self.next_state("position_head")

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(self.start_shoot_angle)

        if self.lobras_head.is_ready(acceptable_error=2):
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
    def prepare_to_fire(self):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        self.shooter.shoot_speaker()

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
        if self.lobras_arm.is_ready(acceptable_error=2):
            self.next_state("position_head")

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(self.head_angle)

        if self.lobras_head.is_ready(acceptable_error=2):
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

    @timed_state(
        first=True, duration=0.5, must_finish=True, next_state="stop_and_retract"
    )
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
    arm_angle = tunable(98)
    head_angle = tunable(180)
    head_shoot_angle = tunable(111)
    drivetrain: SwerveDrive
    ready_to_fire = False
    arduino_light: arduino_light.I2CArduinoLight
    actionStow: ActionStow

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def rotate(self):
        self.ready_to_fire = False

        self.drivetrain.snap_angle(geometry.Rotation2d.fromDegrees(-90))  # We throw from behind
        if self.drivetrain.angle_reached():
            self.next_state("prepare_to_shoot")

    @state
    def prepare_to_shoot(self):
        self.lobras_head.set_angle(self.head_angle)
        self.lobras_arm.set_angle(self.arm_angle)
        self.shooter.shoot_amp()
        self.drivetrain.snap_angle(geometry.Rotation2d.fromDegrees(-90))  # We throw from behind

        if not self.lobras_arm.is_ready(acceptable_error=2):
            return
        if not self.lobras_head.is_ready(acceptable_error=2):
            return
        self.next_state("wait_release")

    @state
    def wait_release(self):
        self.drivetrain.snap_angle(geometry.Rotation2d.fromDegrees(-90))  # We throw from behind
        self.ready_to_fire = True
        self.arduino_light.set_RGB(255, 0, 0)

    @state(must_finish=True)
    def place_head(self):
        self.lobras_head.set_angle(self.head_shoot_angle)

        if self.lobras_head.is_ready(acceptable_error=2):
            self.next_state("feed")

    @timed_state(duration=0.5, must_finish=True)
    def feed(self):
        self.intake.feed()

    def done(self) -> None:
        if self.ready_to_fire:
            self.ready_to_fire = False
            self.next_state("place_head")
        else:
            self.actionStow.engage()
            return super().done()


class ActionShootAmpAuto(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    arm_angle = tunable(98)
    head_angle = tunable(180)
    head_shoot_angle = tunable(111)
    ready_to_fire = False
    arduino_light: arduino_light.I2CArduinoLight
    actionStow: ActionStow

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def prepare_to_shoot(self):
        self.lobras_head.set_angle(self.head_angle)
        self.lobras_arm.set_angle(self.arm_angle)
        self.shooter.shoot_amp()

        if not self.lobras_arm.is_ready(acceptable_error=5):
            return
        if not self.lobras_head.is_ready(acceptable_error=5):
            return
        self.next_state("place_head")

    @state(must_finish=True)
    def place_head(self):
        self.lobras_head.set_angle(self.head_shoot_angle)

        if self.lobras_head.is_ready(acceptable_error=5):
            self.next_state("feed")

    @timed_state(duration=0.5, must_finish=True)
    def feed(self):
        self.intake.feed()

    def done(self) -> None:
        self.actionStow.engage()
        return super().done()


class ActionLowShootAuto(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    start_shoot_angle = tunable(50)
    drivetrain: SwerveDrive
    field_layout: FieldLayout
    is_sim: bool
    arduino_light: arduino_light.I2CArduinoLight
    actionStow: ActionStow
    ARM_OFFSET = tunable(0)
    _teleop = False

    @feedback
    def get_best_angle(self):
        speaker_position = self.field_layout.getSpeakerRelativePosition()
        if speaker_position is None:
            return
        distance = (
            math.sqrt(speaker_position.x**2 + speaker_position.y**2)
        )
        angle = tools.calculate_optimal_launch_angle(
            distance, 0, 0
        )
        return [distance, angle]

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state
    def manual(self):
        self._teleop = True
        self.next_state_now("position_arm")

    @state(first=True)
    def position_arm(self, initial_call):
        if self.intake.has_object() is False and initial_call:
            self.intake.jiggle()
            return

        if self.intake.is_executing:
            return

        # if self.intake.has_object() is False:
        #     self.next_state("finish")
        #     return

        self.lobras_arm.set_angle(self.ARM_OFFSET)
        self.lobras_head.set_angle(70)
        if self.lobras_arm.is_ready(acceptable_error=2) or self.is_sim:
            self.next_state("prepare_to_fire")

    # @state
    def set_launch_rotation(self):
        speaker_position = self.field_layout.getSpeakerRelativePosition()
        if speaker_position is None:
            return

        # Rotate the robot
        target_angle = tools.compute_angle(speaker_position.X(), speaker_position.Y())
        self.drivetrain.snap_angle(geometry.Rotation2d.fromDegrees(target_angle + 180))  # We throw from behind
        if self.drivetrain.angle_reached(acceptable_error=2):
            self._launch_rotation = target_angle
            return True
            # self.next_state("set_launch_angle")
        return False

    # @state
    def set_launch_angle(self):
        speaker_position = self.field_layout.getSpeakerRelativePosition()
        if speaker_position is None:
            return

        # Set the head angle
        distance = math.sqrt(speaker_position.x**2 + speaker_position.y**2)
        angle = tools.calculate_optimal_launch_angle(
            distance,
            0,
            0,
        )

        self.lobras_head.set_angle(angle)
        if self.lobras_head.is_ready(acceptable_error=2):
            return True
            # self.next_state("prepare_to_fire")
        return False

    @timed_state(duration=4, next_state="fire")
    def prepare_to_fire(self):
        res1 = self.set_launch_rotation()
        res2 = self.set_launch_angle()
        self.shooter.shoot_speaker()
        if self.shooter.is_ready() and res1 and res2:
            self.next_state_now("fire")

    @timed_state(must_finish=True, duration=0.7, next_state="finish")
    def fire(self):
        self.intake.feed()

    @state
    def finish(self):
        self.intake.disable()
        self.shooter.disable()
        self.lobras_head.set_angle(0)
        self.arduino_light.set_RGB(0, 255, 0)
        if self._teleop is False:
            self.done()

    def done(self) -> None:
        self.actionStow.engage()
        return super().done()


class ActionHighShootAuto(ActionLowShootAuto):
    Z_OFFSET = tunable(1.03)
    X_OFFSET = tunable(-0.4)
    THROW_VELOCITY = tunable(10.7)
    ARM_OFFSET = tunable(100)
    THROW_OFFSET = tunable(140)

    @timed_state(duration=2, next_state="fire")
    def prepare_to_fire(self):
        res1 = self.set_launch_rotation()
        res2 = self.set_launch_angle()
        self.shooter.shoot_speaker()
        if self.shooter.is_ready() and res1 and res2:
            self.next_state_now("fire")

    @timed_state(must_finish=True, duration=0.5, next_state="finish")
    def fire(self):
        self.intake.feed()


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
        if self.lobras_arm.is_ready(acceptable_error=2):
            self.next_state("position_head")

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(self.head_angle)

        if self.lobras_head.is_ready(acceptable_error=2):
            self.next_state("dewinch")

    @state
    def dewinch(self):
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
        self.drivetrain.snap_angle(geometry.Rotation2d.fromDegrees(target_rotation))
        if self.drivetrain.angle_reached(acceptable_error=2):
            self.next_state("position_arm")

    @state
    def position_arm(self):
        self.lobras_arm.set_angle(self.ARM_ANGLE)
        if self.lobras_arm.is_ready(acceptable_error=2):
            self.next_state("position_head")

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(self.HEAD_ANGLE)

        if self.lobras_head.is_ready(acceptable_error=2):
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


class ActionLowShootTune(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    shoot_angle = tunable(79)
    actionStow: ActionStow

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)


    @state(first=True)
    def insert_note(self):
        if not self.intake.has_object():
            self.intake.intake()
        else:
            self.intake.jiggle()
            self.next_state("position_arm")

    @state
    def position_arm(self):
        self.lobras_arm.set_angle(0)
        if self.lobras_arm.is_ready(acceptable_error=2):
            self.next_state("position_head")

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(self.shoot_angle)

        if self.lobras_head.is_ready(acceptable_error=2):
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
    def prepare_to_fire(self):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        self.shooter.shoot_speaker()

        if self.shooter.is_ready():
            self.next_state("feed_start")

    @timed_state(duration=1.5)
    def feed_start(self, initial_call):
        if initial_call:
            self.intake.feed()

    def done(self) -> None:
        self.actionStow.engage()
        return super().done()