
import wpilib
import rev

class LoBras:
    head_motor: rev.CANSparkMax
    arm_motor: rev.CANSparkMax
    arm_limit_switch: wpilib.DigitalInput
    pneumatic_brake: wpilib.Solenoid

    def setup(self):
        pass
        # TODO
        # Aller chercher l'encoder absolue des 2 moteurs

    def set_angle(self, arm_position, head_position):
        # TODO
        # Move the head and arm to a position
        pass

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
        pass



    def note_trap_mode(self):
        # TODO
        # Place la tête et le bras en position note_trap
        pass

    def execute(self):
        # TODO
        pass
