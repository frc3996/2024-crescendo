
import wpilib
import phoenix6

class Intake:
    input_motor: phoenix6.hardware.TalonFX
    output_motor: phoenix6.hardware.TalonFX
    beam_sensor: wpilib.DigitalInput

    def setup(self):
        pass

    def intake(self):
        # TODO
        # Si appelé, intake disque, sauf si beam sensor activé

    def output(self, fire=False):
        # TODO
        # Lorsqu'appellé, accélère les roues

        if fire is False:
            return

        # Si appelé, tire si bonne vitesse

    def execute(self):
        # TODO
        pass
