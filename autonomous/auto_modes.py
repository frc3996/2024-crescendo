import ntcore
from components import swervedrive
from magicbot.state_machine import timed_state

from .base_auto import BaseAuto


class AvanceEtRecule(BaseAuto):
    MODE_NAME = "AvanceEtRecule"
    DEFAULT = True

    # Injection
    drive: swervedrive.SwerveDrive

    @timed_state(duration=1, next_state="avance", first=True)
    def init(self):
        # Ne pas oublier de pré-définir les variables dans la fonction 'robotInit' de robot.py
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("robotpy")
        self.skip_recule = self.nt.getNumber("auto_mode/skip_recule", False)

    @timed_state(duration=1, next_state="recule")
    def avance(self):
        self.drive.move(0.1, 0)

    @timed_state(duration=1, next_state="finish")
    def recule(self):
        if self.skip_recule is True:
            # Il est possible de forcer le changement immédiat vers un autre état avec:
            self.next_state("finish")

        self.drive.move(-0.1, 0)
