
import ntcore


class Limelight:
    def __init__(self, limelight_name="limelight"):
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable(limelight_name)

    def get_tx(self):
        return self.nt.getNumber('tx', 0)

    def get_ty(self):
        return self.nt.getNumber('ty', 0)

    def get_ta(self):
        return self.nt.getNumber('ta', 0)

    def set_pipeline(self, pipeline_no):
        self.nt.putNumber('pipeline', pipeline_no)

    def light_on(self):
        self.nt.putNumber('ledMode', 0)

    def light_off(self):
        self.nt.putNumber('ledMode', 1)

    def is_target_in_view(self):
        return self.nt.getNumber('tv', 0) == 1

    def is_target_ready(self, acceptable_offset=2):
        if self.is_target_in_view() is False:
            return False

        if abs(self.get_tx()) < acceptable_offset:
            return True
        return False
