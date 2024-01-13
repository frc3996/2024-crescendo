
import wpilib
import phoenix6

class LoBras:
    head_motor: phoenix6.hardware.TalonFX
    arm_motor: phoenix6.hardware.TalonFX
    arm_limit_switch: wpilib.DigitalInput
    pneumatic_brake: wpilib.Solenoid

    def setup(self):
        pass
        # TODO
        # Aller chercher l'encoder absolue des 2 moteurs

    def __set_positions(self, head_position, arm_position):
        # TODO
        # Move the head and arm to a position

    def intake_mode(self):
        # TODO
        # Place la tête et le bras en position intake
        self.__set_positions(xx, yy)

    def speaker_mode(self, fire=False):
        # TODO
        # Place le bras en mode speaker
        # Utilise la limelight pour viser avec la tete

        if fire is False:
            return

        # Tire si limelight + vitesse Ok

    def amp_mode(self):
        # TODO
        # Place la tête et le bras en position amp



    def note_trap_mode(self):
        # TODO
        # Place la tête et le bras en position note_trap

    def execute(self):
        # TODO
