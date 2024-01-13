
import math
from components import swervedrive, lobra, intake


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

    def setup(self):
        pass

    def autoshoot_amp(self):


        self.drivetrain.set_angle(AMP_ANGLE)
        if not self.limelight.check_tag_in_view(AMP_TAG):
            self.drive.controller_drive()
            return

        self.drivetrain.limelight_drive()

        if not self.limelight.check_tag_ready(AMP_TAG):
            return

    # def autoshoot_speaker(self):
    #     pass

    def retract(self):
        pass

    def pixie_intake(self):
        arm_angle = 20  # TODO calibrate all
        head_angle = 30  # TODO calibrate all

        self.intake.intake()
        self.lobras.set_angle(arm_angle, head_angle)

        self.intake_limelight.set_cube_mode()
        if self.intake_limelight.get_target_valid():
            offset = self.intake_limelight.get_tx()
            res = get_linear_damp_ratio(abs(offset), 50, 0)
            fwd = 0.5 * res
            self.intake_limelight_adjust_pid.setP(self.nt.getNumber("claw/limelight_adjust_pid/Kp", 0))
            self.intake_limelight_adjust_pid.setI(self.nt.getNumber("claw/limelight_adjust_pid/Ki", 0))
            self.intake_limelight_adjust_pid.setD(self.nt.getNumber("claw/limelight_adjust_pid/Kd", 0))
            error = self.intake_limelight_adjust_pid.calculate(offset, 0)
            # self.last_valid_cube = time.time()
        else:
            fwd = 0.5
            error = 0

        self.drivetrain.set_relative_automove_value(fwd, error)

        return False
