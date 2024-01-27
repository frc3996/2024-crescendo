import wpilib  # Librairie de base de la FRC
import ntcore


class Pixy:
    def __init__(self, analogPort=0, digitalPort=0):
        self.offset = wpilib.AnalogInput(analogPort)
        self.isTarget = wpilib.DigitalInput(digitalPort)

    def get_target_valid(self):
        return self.isTarget.get()

    def get_offset(self):
        """Returns offset value"""
        return self.offset.getValue() - 1350

    def targetReady(self, acceptable_offset=0.01):
        if self.get_target_valid() is False:
            return False

        if abs(self.get_offset()) < acceptable_offset:
            return True
        return False

    def update_sd(self, network_table: ntcore.NetworkTable):
        network_table.putNumber("pixy/valid", self.get_target_valid())
        network_table.putNumber("pixy/offset", self.get_offset())
