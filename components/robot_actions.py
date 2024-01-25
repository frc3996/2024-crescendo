
import wpilib
import ntcore
from wpimath import controller
from components import swervedrive, lobra, intake
from common import limelight, arduino_light, path_helper, tools


class RobotActions:
    drivetrain: swervedrive.SwerveDrive
    intake: intake.Intake
    lobras: lobra.LoBras
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
            return

        self.drivetrain.set_angle(0) # TODO GET PROPER ANGLE
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
            self.shoot_limelight_adjust_pid.setP(self.nt.getNumber("actions/shoot_limelight_adjust_pid/Kp", 0))
            self.shoot_limelight_adjust_pid.setI(self.nt.getNumber("actions/shoot_limelight_adjust_pid/Ki", 0))
            self.shoot_limelight_adjust_pid.setD(self.nt.getNumber("actions/shoot_limelight_adjust_pid/Kd", 0))
            error = self.shoot_limelight_adjust_pid.calculate(offset, 0)
        else:
            fwd = 0.5
            error = 0

        self.drivetrain.set_relative_automove_value(fwd, error)

    def autoshoot_speaker(self):
        pass

    def retract(self):
        arm_angle = 0  # TODO calibrate all
        head_angle = 0  # TODO calibrate all
        self.lobras.set_angle(arm_angle, head_angle)
        self.shoot_limelight_adjust_pid.reset()
        self.intake_limelight_adjust_pid.reset()
        pass

    def TestLoBra(self, angle):
        self.lobras.set_arm_angle(angle)

    def autointake_with_limelight(self):
        arm_angle = 20  # TODO calibrate all
        head_angle = 30  # TODO calibrate all

        if self.intake.is_object_intaken() is True:
            self.arduino_light.set_RGB(0, 0, 255)
            # self.status_light.set(1)
            self.retract()
            return

        self.intake.intake()
        self.lobras.set_angle(arm_angle, head_angle)
        if self.limelight_intake.is_target_in_view():
            offset = self.limelight_intake.get_tx()
            res = tools.map_value(abs(offset), 0, 50, 0, 1)
            fwd = 0.5 * res
            self.intake_limelight_adjust_pid.setP(self.nt.getNumber("actions/intake_limelight_adjust_pid/Kp", 0))
            self.intake_limelight_adjust_pid.setI(self.nt.getNumber("actions/intake_limelight_adjust_pid/Ki", 0))
            self.intake_limelight_adjust_pid.setD(self.nt.getNumber("actions/intake_limelight_adjust_pid/Kd", 0))
            error = self.intake_limelight_adjust_pid.calculate(offset, 0)
        else:
            fwd = 0.5
            error = 0

        self.drivetrain.relative_rotate(-error)
        self.drivetrain.set_relative_automove_value(fwd, 0)

    def execute(self):
        pass
