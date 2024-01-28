import rev
import wpilib

import constants

INTAKE_SPEED = 10


# TODO MOVE TO INTAKE
class IntakeBeam:
    def setup(self):
        self.beam = wpilib.AnalogInput(2)

    # Informational methods
    def object_in_sensor(self):
        return self.beam.getValue() < 324

    def execute(self):
        pass


class Intake:
    def setup(self):
        self.input_motor = rev.CANSparkMax(
            constants.CANIds.INTAKE, rev.CANSparkMax.MotorType.kBrushless
        )

    def enable(self):
        pass

    def is_object_intaken(self):
        return False

    def feed_shooter(self):
        pass

    def disable(self):
        pass

    def execute(self):
        pass
