
import ntcore
import math

CUBE_MODE_RIGHTMOST = 3
CUBE_MODE_LEFTMOST = 4
CUBE_MODE_CLOSEST = 5

class Limelight:
    def __init__(self, limelight_name="limelight"):
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable(limelight_name)
        self.current_pipeline = -1
        self.cube_position = CUBE_MODE_RIGHTMOST

    def get_target_valid(self):
        return self.nt.getNumber('tv', 0) == 1

    def get_tx(self):
        return self.nt.getNumber('tx', 0)

    def get_ty(self):
        return self.nt.getNumber('ty', 0)

    def get_ta(self):
        return self.nt.getNumber('ta', 0)

    def set_reflective_mode(self):
        # if self.current_pipeline == 0:
        #     return
        # self.current_pipeline = 0
        self.nt.putNumber('pipeline', 0)

    def set_april_tag_mode(self):
        # if self.current_pipeline == 1:
        #     return
        # self.current_pipeline = 1
        self.nt.putNumber('pipeline', 1)

    def set_cone_mode(self):
        # if self.current_pipeline == 2:
        #     return
        # self.current_pipeline = 2
        self.nt.putNumber('pipeline', 2)

    def set_cube_mode(self):
        # if self.current_pipeline == 2:
        #     return
        # self.current_pipeline = 2
        self.nt.putNumber('pipeline', self.cube_position)

    def set_cube_position(self, position):
        self.cube_position = position

    def light_on(self):
        self.nt.putNumber('ledMode', 0)

    def light_off(self):
        self.nt.putNumber('ledMode', 1)

    def targetReady(self, acceptable_offset=2):
        if self.get_target_valid() is False:
            return False
        
        if abs(self.get_target_x_offset()) < acceptable_offset:
            return True
        return False
