import wpilib
import struct


class I2CArduinoLight:
    def __init__(self, i2c_port: wpilib.I2C.Port, i2c_id):
        self.i2c_device = wpilib.I2C(i2c_port, i2c_id)

    def set_RGB(self, red, green, blue):
        """
        Set RGB color. RBG Value from 0 to 255
        """
        red = max(0, min(255, red))
        green = max(0, min(255, green))
        blue = max(0, min(255, blue))

        data = struct.pack("BBB", red, green, blue)
        self.i2c_device.writeBulk(data)
