
import wpilib
import phoenix6

class Intake:
    input_motor: phoenix6.hardware.TalonFX
    output_motor: phoenix6.hardware.TalonFX
    beam_sensor: wpilib.DigitalInput

    def setup(self):
        self.input_motor = phoenix6.hardware.TalonFX(2)
        self.output_motor = phoenix6.hardware.TalonFX(3)
        self.output_motor = wpilib.DigitalInput(0)

    def intake(self):
        # TODO
        # Si appelé, intake disque, sauf si beam sensor activé
        pass

    def output(self, fire=False):
        # TODO
        # Lorsqu'appellé, accélère les roues

        if fire is False:
            return

        # Si appelé, tire si bonne vitesse

    def execute(self):
        # TODO
        pass
