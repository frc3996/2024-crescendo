
from components.intake import Intake
from components.lobra import LoBras

class Automations:
    intake: Intake
    lobras: LoBras
    drivetrain: DriveTrain

    def setup(self):
        pass

    def auto_shoot_amp(self):
        self.drivetrain.set_angle(AMP_ANGLE)
        if not self.limelight.check_tag_in_view(AMP_TAG):
            self.drive.controller_drive()
            return

        self.drivetrain.limelight_drive()

        if not self.limelight.check_tag_ready(AMP_TAG):
            return

        self.lobras.amp_mode()
        # Limelight check pour le tag
        # Si tag pas vue:
            #

    def auto_shoot_speaker(self):
        pass
