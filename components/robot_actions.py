
from components import swervedrive, lobra, intake
from wpimath.controller import PIDController
from common import limelight
import ntcore


def get_linear_damp_ratio(current_value, minimum, maximum):
    """Return a ratio from 1 to 0 based on a range"""
    m = maximum - minimum
    b = minimum
    res = (current_value - b) / m
    res = min(1, res)
    res = max(0, res)
    return res


class RobotActions:
    drivetrain: swervedrive.SwerveDrive
    intake: intake.Intake
    lobras: lobra.LoBras
    limelight_intake: limelight.Limelight
    limelight_shoot: limelight.Limelight
    nt: ntcore.NetworkTable

    def setup(self):
        self.nt.putNumber("actions/shoot_limelight_adjust_pid/Kp", 0.012)
        self.nt.putNumber("actions/shoot_limelight_adjust_pid/Ki", 0)
        self.nt.putNumber("actions/shoot_limelight_adjust_pid/Kd", 0)
        self.shoot_limelight_adjust_pid = PIDController(0, 0, 0)

        self.nt.putNumber("actions/intake_limelight_adjust_pid/Kp", 0.012)
        self.nt.putNumber("actions/intake_limelight_adjust_pid/Ki", 0)
        self.nt.putNumber("actions/intake_limelight_adjust_pid/Kd", 0)
        self.intake_limelight_adjust_pid = PIDController(0, 0, 0)

    def autoshoot_amp(self):
        arm_angle = 0  # TODO calibrate all
        head_angle = 0  # TODO calibrate all
        if self.intake.shot_done is True:
            self.retract()
            return

        self.drivetrain.set_angle(0) # TODO GET PROPER ANGLE
        self.lobras.set_angle(arm_angle, head_angle)

        if self.limelight_intake.is_target_in_view():
            offset = self.limelight_intake.get_tx()
            if self.limelight_intake.get_ta() >= 1.5:  # TODO ADJUST SIZE TO ARRIVAL
                self.intake.shoot(fire=True)
            else:
                # Move forward
                res = get_linear_damp_ratio(abs(offset), 50, 0)
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

    def autointake_with_limelight(self):
        arm_angle = 20  # TODO calibrate all
        head_angle = 30  # TODO calibrate all

        if self.intake.note_intaken() is True:
            self.retract()
            return

        self.intake.intake()
        self.lobras.set_angle(arm_angle, head_angle)

        if self.limelight_intake.is_target_in_view():
            offset = self.limelight_intake.get_tx()
            res = get_linear_damp_ratio(abs(offset), 50, 0)
            fwd = 0.5 * res
            self.intake_limelight_adjust_pid.setP(self.nt.getNumber("actions/intake_limelight_adjust_pid/Kp", 0))
            self.intake_limelight_adjust_pid.setI(self.nt.getNumber("actions/intake_limelight_adjust_pid/Ki", 0))
            self.intake_limelight_adjust_pid.setD(self.nt.getNumber("actions/intake_limelight_adjust_pid/Kd", 0))
            error = self.intake_limelight_adjust_pid.calculate(offset, 0)
        else:
            fwd = 0.5
            error = 0

        self.drivetrain.set_relative_automove_value(fwd, error)

    def execute(self):
        pass
