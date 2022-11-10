import adafruit_vl6180x
import board
import busio
from smbus import SMBus as Bus

class LidarSensor:

    def __init__(self, port, threshold=25, address_offset=0b000, address_default=0x70):
        self.port = port
        self.threshold = threshold
        self.delay_time = 0.001
        self.address = address_offset + address_default
        self.detected = False
        self.last_read = self.threshold + 10
        self.i2c_smbus = Bus(1)
        self.i2c_smbus.write_byte(self.address, 1 << self.port)
        self.i2c_circuit_python = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_vl6180x.VL6180X(self.i2c_circuit_python)

    def distance(self):
        self.i2c_smbus.write_byte(self.address, 1 << self.port)
        self.last_read = self.sensor.range
        return self.last_read