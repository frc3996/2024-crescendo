import math

import numpy
import wpilib
import wpimath
import wpimath.units
from magicbot import StateMachine, feedback, state, timed_state, tunable
from magicbot.state_machine import StateRef
from wpimath import controller, geometry

import constants
from common import tools
from common.arduino_light import I2CArduinoLight, LedMode
from common.path_helper import PathHelper
from components import intake, swervedrive
from components.climber import Climber
from components.field import FieldLayout
from components.intake import Intake
from components.limelight import LimeLightVision
from components.lobra import LoBrasArm, LoBrasHead
from components.pixy import Pixy
from components.shooter import Shooter
from components.swervedrive import SwerveDrive


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

    @timed_state(first=True, must_finish=True, duration=2)
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

        if self.lobras_arm.is_ready(acceptable_error=29):
            self.done()

    def done(self):
        super().done()


class ActionGrabAuto(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    intake: Intake
    arduino_light: I2CArduinoLight
    drivetrain: SwerveDrive
    pixy: Pixy
    auto_intake_kp = tunable(0.001)  # For relative_rotate: 0.015
    auto_intake_ki = tunable(0)  # For relative_rotate: 0
    auto_intake_kd = tunable(0)  # For relative_rotate: 0
    auto_intake_pid = controller.PIDController(0, 0, 0)
    intake_target_angle = tunable(111)
    intake_target_speed = tunable(1.25)
    actionStow: ActionStow
    limelight_vision: LimeLightVision

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def position_head(self):
        """Premier etat, position la tete"""
        if self.intake.has_object():
            self.limelight_vision.light_blink()
            self.next_state("finish")
            return

        self.arduino_light.set_leds(LedMode.BlinkSlow, 0, 255, 0)

        self.auto_intake_pid.setPID(
            self.auto_intake_kp,
            self.auto_intake_ki,
            self.auto_intake_kd,
        )
        self.auto_intake_pid.reset()

        self.lobras_arm.set_angle(0)
        self.lobras_head.set_angle(self.intake_target_angle)

        self.next_state("start_intake")

    @state
    def start_intake(self, initial_call):
        if initial_call:
            self.timer = wpilib.Timer()
            self.timer.start()

        if tools.is_autonomous() and self.timer.get() > 2:
            self.next_state("finish")

        if self.intake.has_object():
            self.arduino_light.set_leds(LedMode.BlinkFast, 0, 255, 0)
            self.limelight_vision.light_blink()
            self.next_state("finish")

        if initial_call:
            self.intake.intake()

        # # Automove to target
        if self.pixy.get_target_valid():
            offset = self.pixy.get_offset()
            res = tools.map_value(abs(offset), 0, 1000, 0, 0.5)
            fwd = self.intake_target_speed * (1 - res)
            error = self.auto_intake_pid.calculate(offset, 0)
        else:
            fwd = 0.35
            if tools.is_autonomous():
                fwd = 0.6
            error = 0
        self.drivetrain.set_robot_relative_automove_value(-fwd, -error)

    @state
    def finish(self, initial_call):
        if tools.is_autonomous():
            self.intake.jiggle()
            self.done()
            return
        if initial_call:
            self.lobras_head.set_angle(70)
            self.intake.jiggle()

    def done(self) -> None:
        self.arduino_light.set_leds(LedMode.Solid, 0, 255, 0)
        self.limelight_vision.light_off()
        if not tools.is_autonomous():
            self.actionStow.engage()
        return super().done()


AMP_ARM_ANGLE = 98
AMP_HEAD_ANGLE = 180
AMP_SHOOT_ANGLE = 111


class ActionShootAmpAssisted(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    arm_angle = tunable(AMP_ARM_ANGLE)
    head_angle = tunable(AMP_HEAD_ANGLE)
    head_shoot_angle = tunable(AMP_SHOOT_ANGLE)
    drivetrain: SwerveDrive
    ready_to_fire = False
    arduino_light: I2CArduinoLight
    actionStow: ActionStow

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def rotate(self):
        # Prevent the robot from spinning fast with its arm up
        # TODO: Is this valid with Red team?
        self.drivetrain.snap_angle(
            geometry.Rotation2d.fromDegrees(-90)
        )  # We throw from behind
        self.arduino_light.set_leds(LedMode.BlinkFast, 0, 255, 0)

        self.ready_to_fire = False

        if self.drivetrain.angle_reached():
            self.next_state("prepare_to_shoot")

    @state
    def prepare_to_shoot(self):
        # self.lobras_head.set_angle(self.head_angle)
        self.lobras_head.set_angle(self.head_shoot_angle)
        self.lobras_arm.set_angle(self.arm_angle)
        self.drivetrain.snap_angle(
            geometry.Rotation2d.fromDegrees(-90)
        )  # We throw from behind
        if not self.lobras_arm.is_ready(acceptable_error=5):
            return
        if not self.lobras_head.is_ready(acceptable_error=5):
            return
        self.shooter.shoot_amp()
        self.next_state("wait_release")

    @state
    def wait_release(self):
        self.drivetrain.snap_angle(
            geometry.Rotation2d.fromDegrees(-90)
        )  # We throw from behind
        self.arduino_light.set_leds(LedMode.Swipe, 255, 165, 0)
        self.ready_to_fire = True

    @state(must_finish=True)
    def place_head(self):
        self.drivetrain.snap_angle(
            geometry.Rotation2d.fromDegrees(-90)
        )  # We throw from behind
        self.lobras_head.set_angle(self.head_shoot_angle)
        if self.lobras_head.is_ready(acceptable_error=5):
            self.next_state("feed")

    @timed_state(duration=0.5, must_finish=True)
    def feed(self):
        self.intake.feed()

    def done(self) -> None:
        if self.ready_to_fire:
            self.ready_to_fire = False
            self.next_state("place_head")
        else:
            self.arduino_light.set_leds(LedMode.Solid, 0, 128, 0)
            self.actionStow.engage()
            return super().done()


class ActionShootAmpAuto(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    arm_angle = tunable(AMP_ARM_ANGLE)
    head_angle = tunable(AMP_HEAD_ANGLE)
    head_shoot_angle = tunable(AMP_SHOOT_ANGLE)
    actionStow: ActionStow

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def prepare_to_shoot(self):
        # self.lobras_head.set_angle(self.head_angle)
        self.lobras_head.set_angle(self.head_shoot_angle)
        self.lobras_arm.set_angle(self.arm_angle)
        self.shooter.shoot_amp()
        if not self.lobras_arm.is_ready(acceptable_error=5):
            return
        if not self.lobras_head.is_ready(acceptable_error=5):
            return
        self.next_state("feed")

    @timed_state(duration=0.5, must_finish=True, next_state="feed_stuck")
    def feed(self):
        self.intake.feed()

    @timed_state(duration=4)
    def feed_stuck(self):
        if self.intake.has_object():
            # Lets move at 2 degrees per second
            self.lobras_arm.set_angle(self.lobras_arm.get_angle() - 2 / 50)
            self.lobras_head.set_angle(self.lobras_head.get_angle() + 2 / 50)
            return
        self.done()

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
    arduino_light: I2CArduinoLight
    actionStow: ActionStow
    ARM_ANGLE = tunable(0)
    FUDGE_FACTOR = tunable(0.7)
    THROW_OFFSET = tunable(59.5)

    DISTANCE_POINTS = [1.21, 1.85, 2.85, 3.33, 3.85, 4.85, 5.85]
    ANGLE_POINTS = [96, 87, 79, 77, 73, 70.9, 70.9]

    def get_launch_angle(self, distance):
        return numpy.interp(distance, self.DISTANCE_POINTS, self.ANGLE_POINTS).item()

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @feedback
    def get_best_angle(self):
        speaker_position = self.field_layout.getSpeakerRelativePosition()
        if speaker_position is None:
            return [0, 0]
        distance = math.sqrt(speaker_position.x**2 + speaker_position.y**2)
        angle = self.get_launch_angle(distance)
        return [distance, angle]

    @state(first=True)
    def jiggle(self, initial_call):
        self.arduino_light.set_leds(LedMode.BlinkSlow, 0, 255, 0)
        if self.intake.is_executing:
            return

        if self.intake.has_object() is False and initial_call:
            self.intake.jiggle()
            return

        # self.next_state("prepare_to_fire_basic")
        self.next_state("prepare_to_fire_fancy")

    def set_arm(self):
        self.lobras_arm.set_angle(self.ARM_ANGLE)
        if self.lobras_arm.is_ready(acceptable_error=4) or self.is_sim:
            return True
        return False

    def set_launch_rotation(self):
        speaker_position = self.field_layout.getSpeakerRelativePosition()
        if speaker_position is None:
            return 0

        # Rotate the robot
        target_angle = tools.compute_angle(speaker_position.X(), speaker_position.Y())
        self.drivetrain.snap_angle(
            geometry.Rotation2d.fromDegrees(target_angle)
        )  # We throw from behind
        if self.drivetrain.angle_reached(acceptable_error=4):
            self._launch_rotation = target_angle
            return 1
            # self.next_state("set_launch_angle")
        return 0

    def set_launch_angle(self):
        speaker_position = self.field_layout.getSpeakerRelativePosition()
        if speaker_position is None:
            return 0

        # Set the head angle
        distance = math.sqrt(speaker_position.x**2 + speaker_position.y**2)
        angle = self.get_launch_angle(distance)

        self.lobras_head.set_angle(angle)
        if self.lobras_head.is_ready(acceptable_error=4):
            return 1
            # self.next_state("prepare_to_fire")
        return 0

    def prepare_shooter(self):
        self.shooter.shoot_speaker()
        res = self.shooter.is_ready()
        return 1 if res else 0

    @timed_state(duration=2, next_state="fire")
    def prepare_to_fire_basic(self):
        res1 = self.set_launch_rotation()
        res2 = self.set_launch_angle()
        res3 = self.prepare_shooter()
        res4 = self.set_arm()
        print(res1, res2, res3, res4)
        if res1 == 1 and res2 == 1 and res3 == 1 and res4 == 1:
            self.next_state_now("fire")

    def aim(self, enable_shooter=True):
        if enable_shooter:
            self.shooter.shoot_speaker()

        speaker_position = self.field_layout.getSpeakerRelativePosition(0.40)
        if speaker_position is None:
            return

        projectile_velocity = (
            math.pi
            * (constants.SHOOTER_WHEEL_DIAMETER)
            * (self.shooter.get_velocity() / 60)
            * self.FUDGE_FACTOR
        )

        angle, rotation = tools.get_projectile_launch_angle_and_rotation(
            speaker_position,
            projectile_velocity,
            self.drivetrain.get_chassis_speed(),
            self.drivetrain.getRotation2d(),
        )

        self.drivetrain.snap_angle(
            geometry.Rotation2d.fromDegrees(rotation + 180)
        )  # We throw from behind

        if angle is None:
            return
        angle = math.degrees(angle) + self.THROW_OFFSET
        print(angle, rotation)
        self.drivetrain.set_tmp_speed_factor(factor_rotation=1)
        self.lobras_head.set_angle(angle)

    @timed_state(duration=10, next_state="finish")
    def prepare_to_fire_fancy(self):
        self.aim()
        res1 = self.lobras_head.is_ready(acceptable_error=3)
        res2 = self.drivetrain.angle_reached(acceptable_error=2)
        res3 = abs(self.lobras_head.encoder.getVelocity()) < 0.015
        res4 = abs(self.drivetrain._chassis_speed.omega) < 0.0075
        res5 = self.set_arm()
        res = [res1, res2, res3, res4, res5]
        self.arduino_light.set_leds(LedMode.BlinkFast, 0, 255, 0)
        if all(res):
            self.next_state_now("fire_until_out")

    @timed_state(must_finish=True, duration=0.7, next_state="finish")
    def fire(self):
        self.intake.feed()

    @timed_state(must_finish=True, duration=2, next_state="fire")
    def fire_until_out(self):
        # Keep on aiming
        self.aim()
        self.intake.feed()
        self.arduino_light.set_leds(LedMode.Swipe, 255, 165, 0)
        if not self.intake.has_object():
            self.next_state_now("fire")

    @state
    def finish(self):
        self.intake.disable()
        self.shooter.disable()
        if wpilib.DriverStation.isAutonomous():
            self.done()
            return
        self.lobras_head.set_angle(0)

    def done(self) -> None:
        self.intake.disable()
        self.shooter.disable()
        self.arduino_light.set_leds(LedMode.Solid, 0, 128, 0)
        if not wpilib.DriverStation.isAutonomous():
            self.actionStow.engage()
        return super().done()


class ActionHighShootAuto(ActionLowShootAuto):
    Z_OFFSET = tunable(1.03)
    X_OFFSET = tunable(-0.4)
    THROW_VELOCITY = tunable(10.7)
    ARM_ANGLE = tunable(100)
    THROW_OFFSET = tunable(152)

    # TODO: NEED TO FILL THESE POINTS
    DISTANCE_POINTS = [1.21, 1.85, 2.85, 3.33, 3.85, 4.85, 5.85]
    ANGLE_POINTS = [96, 86, 78, 77, 74, 74, 70.6]


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


class ActionLowShootTune(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake
    drivetrain: SwerveDrive
    shoot_angle = tunable(79)
    actionStow: ActionStow
    field_layout: FieldLayout
    THROW_OFFSET = tunable(58.5)
    FUDGE_FACTOR = tunable(0.85)

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    def get_aim(self):
        speaker_position = self.field_layout.getSpeakerRelativePosition(0.40)
        if speaker_position is None:
            return

        projectile_velocity = (
            math.pi
            * (constants.SHOOTER_WHEEL_DIAMETER)
            * (self.shooter.get_velocity() / 60)
            * self.FUDGE_FACTOR
        )

        angle, rotation = tools.get_projectile_launch_angle_and_rotation(
            speaker_position,
            projectile_velocity,
            self.drivetrain.get_chassis_speed(),
            self.drivetrain.getRotation2d(),
        )

        # self.drivetrain.snap_angle(
        #     geometry.Rotation2d.fromDegrees(math.degrees(rotation))
        # )  # We throw from behind

        if angle is None:
            return

        # self.lobras_head.set_angle(math.degrees(angle) + self.THROW_OFFSET)
        return [math.degrees(rotation), math.degrees(angle) + self.THROW_OFFSET]

    @feedback
    def get_distance(self):
        speaker_position = self.field_layout.getSpeakerRelativePosition()
        if speaker_position is None:
            return -1
        distance = math.sqrt(speaker_position.x**2 + speaker_position.y**2)
        return distance

    @state(first=True)
    def insert_note(self):
        if not self.intake.has_object():
            self.intake.intake()
        else:
            self.intake.jiggle()
            self.next_state("position_arm")

    @state
    def position_arm(self):
        self.lobras_arm.set_angle(100)  # 100 for highshoot, else 0
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
