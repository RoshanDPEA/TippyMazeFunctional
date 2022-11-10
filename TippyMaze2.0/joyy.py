import sys
from threading import Thread

from LidarSensor import LidarSensor

sys.path.append("/home/pi/packages")
sys.path.append("/home/pi/packages/dpea-odrive")
sys.path.append("/home/pi/packages/RaspberryPiCommon/pidev")
sys.path.append("/home/pi/TippyMaze2.0")

from Joystick import Joystick
from time import sleep
import enum
from dpea_p2p import Server
from dpea_p2p import Client

joy = Joystick(0, False)
joyvalue = None
TOP_RAMP_SENSOR = LidarSensor(port=5, threshold=70)

class PacketType(enum.Enum):
    NULL = 0
    COMMAND1 = 1
    COMMAND2 = 2

c = Client("172.17.21.47", 5001, PacketType)

def joy_listen():
    global joyvalue
    c.connect()
    while True:
        pack = c.recv_packet()
        joyvalue = pack[1].decode()
        sleep(0.05)

if __name__ == "__main__":
    # Thread(target=joy_listen).start()
    while True:
        if TOP_RAMP_SENSOR.distance() <= 60:
            print("TOP")
    #     if joyvalue == "1":
    #         print("DOWN")
        sleep(0.1)