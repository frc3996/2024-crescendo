import ntcore
import wpilib
import math
from magicbot import StateMachine, default_state, feedback, state, timed_state
from wpimath import controller, geometry

from common import arduino_light, limelight, path_helper, tools
from components import *


class RobotActions:
    drivetrain: swervedrive.SwerveDrive
    intake: Intake
    lobras: LoBras
    limelight_intake: limelight.Limelight
    limelight_shoot: limelight.Limelight
    arduino_light: arduino_light.I2CArduinoLight
    status_light: wpilib.Solenoid
    nt: ntcore.NetworkTable

    def setup(self):
        self.nt.putNumber("actions/shoot_limelight_adjust_pid/Kp", 0.012)
        self.nt.putNumber("actions/shoot_limelight_adjust_pid/Ki", 0)
        self.nt.putNumber("actions/shoot_limelight_adjust_pid/Kd", 0)
        self.shoot_limelight_adjust_pid = controller.PIDController(0, 0, 0)

        self.nt.putNumber("actions/intake_limelight_adjust_pid/Kp", 0.7)
        self.nt.putNumber("actions/intake_limelight_adjust_pid/Ki", 0)
        self.nt.putNumber("actions/intake_limelight_adjust_pid/Kd", 0)
        self.intake_limelight_adjust_pid = controller.PIDController(0, 0, 0)

        self.test_path = path_helper.PathHelper(self.drivetrain, "test_path2")

    def reset_auto(self):
        self.must_reset_auto = True

    def auto_test(self):
        if self.must_reset_auto is True:
            self.test_path.init_path(force_robot_starting_position=True)
            self.must_reset_auto = False

        # if self.test_path.path_reached_end():
        #     return

        self.test_path.auto_move()

    def autoshoot_amp(self):
        arm_angle = 0  # TODO calibrate all
        head_angle = 0  # TODO calibrate all
        if self.intake.shot_done is True:
            self.retract()
            return True

        self.drivetrain.set_angle(0)  # TODO GET PROPER ANGLE
        self.lobras.set_angle(arm_angle, head_angle)
        fwd = 0
        error = 0
        if self.limelight_intake.is_target_in_view():
            offset = self.limelight_intake.get_tx()
            if self.limelight_intake.get_ta() >= 1.5:  # TODO ADJUST SIZE TO ARRIVAL
                self.intake.shoot(fire=True)
            else:
                # Move forward
                res = tools.map_value(abs(offset), 0, 50, 0, 1)
                fwd = 0.5 * res
            self.shoot_limelight_adjust_pid.setP(
                self.nt.getNumber("actions/shoot_limelight_adjust_pid/Kp", 0)
            )
            self.shoot_limelight_adjust_pid.setI(
                self.nt.getNumber("actions/shoot_limelight_adjust_pid/Ki", 0)
            )
            self.shoot_limelight_adjust_pid.setD(
                self.nt.getNumber("actions/shoot_limelight_adjust_pid/Kd", 0)
            )
            error = self.shoot_limelight_adjust_pid.calculate(offset, 0)
        else:
            fwd = 0.5
            error = 0

        self.drivetrain.set_relative_automove_value(fwd, error)
        return False

    def autoshoot_speaker(self):
        pass

    def retract(self):
        arm_angle = 0  # TODO calibrate all
        head_angle = 0  # TODO calibrate all
        self.lobras.set_angle(arm_angle, head_angle)
        self.shoot_limelight_adjust_pid.reset()
        self.intake_limelight_adjust_pid.reset()
        pass

    def autointake_with_limelight(self):
        arm_angle = 0  # TODO calibrate all
        head_angle = 0  # TODO calibrate all

        if self.intake.is_object_intaken() is True:
            self.arduino_light.set_RGB(0, 0, 255)
            # self.status_light.set(1)
            self.retract()
            return True

        self.intake.intake()
        self.lobras.set_angle(arm_angle, head_angle)
        if self.limelight_intake.is_target_in_view():
            offset = self.limelight_intake.get_tx()
            res = tools.map_value(abs(offset), 0, 50, 0, 1)
            fwd = 0.5 * res
            self.intake_limelight_adjust_pid.setP(
                self.nt.getNumber("actions/intake_limelight_adjust_pid/Kp", 0)
            )
            self.intake_limelight_adjust_pid.setI(
                self.nt.getNumber("actions/intake_limelight_adjust_pid/Ki", 0)
            )
            self.intake_limelight_adjust_pid.setD(
                self.nt.getNumber("actions/intake_limelight_adjust_pid/Kd", 0)
            )
            error = self.intake_limelight_adjust_pid.calculate(offset, 0)
        else:
            fwd = 0.5
            error = 0

        self.drivetrain.relative_rotate(-error)
        self.drivetrain.set_relative_automove_value(fwd, 0)
        return False

    def execute(self):
        pass


class ActionGrab(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    intake: Intake

    @state(first=True)
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(100)

        if self.lobras_head.is_ready(acceptable_error=10):
            self.next_state("position_arm")

    @state
    def position_arm(self):
        self.lobras_arm.set_angle(0)
        if self.lobras_arm.is_ready():
            self.next_state("start_intake")

    @state
    def start_intake(self):
        self.intake.enable()
        if self.intake.has_object():
            self.done()

    def done(self) -> None:
        self.lobras_head.set_angle(10)
        self.intake.disable()
        return super().done()


class ActionShoot(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake

    @state(first=True)
    def position_arm(self):
        self.lobras_arm.set_angle(100)
        print("arm Current Angle", round(self.lobras_arm.get_angle()), "Target", round(self.lobras_arm._target_position))
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
        print("PREPARE_TO_FIRE")
        self.shooter.enable()
        print(self.shooter.encoder.getVelocity())

        # Use a normal state
        if self.shooter.is_ready():
            self.next_state("feed_start")

    @timed_state(duration=2, next_state="done")
    def feed_start(self):
        print("FEEDING")
        self.shooter.enable()
        self.intake.enable()
        print("HAS_OBJCET", self.intake.has_object())
        if not self.intake.has_object():
            self.done()

    def done(self) -> None:
        print("DONE")
        self.shooter.disable()
        self.intake.disable()
        return super().done()


class ActionStow(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake: Intake

    def fire(self):
        """This function is called from teleop or autonomous to cause the
        shooter to fire"""
        self.engage()

    @state(first=True)
    def position_all(self):
        """Premier etat, position la tete, et on s'assure que plu rien tourne"""
        self.shooter.disable()
        self.intake.disable()

        self.lobras_head.set_angle(0)
        if self.lobras_head.get_angle() < 90:
            self.lobras_arm.set_angle(0)
        else:
            self.lobras_arm.set_angle(30)

        if self.lobras_arm.is_ready() and self.lobras_head.is_ready():
            self.done()
