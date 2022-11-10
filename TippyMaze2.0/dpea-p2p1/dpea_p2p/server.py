import sys

sys.path.append("/home/pi/packages")
sys.path.append("/home/pi/packages/dpea-odrive")
sys.path.append("/home/pi/packages/RaspberryPiCommon/pidev")
sys.path.append("/home/pi/TippyMaze2.0")

import enum
from Joystick import Joystick
from LidarSensor import LidarSensor
from dpea_p2p import Server
from time import sleep
from threading import Thread
from odrive_helpers import *

class PacketType(enum.Enum):
    NULL = 0
    COMMAND1 = 1
    COMMAND2 = 2

joy = Joystick(0, False)
counter = 0
top_sensor = False
bottom_sensor = False

TOP_RAMP_SENSOR = LidarSensor(port=5, threshold=70)
LOWER_RAMP_SENSOR = LidarSensor(port=3, threshold=70)

#          |Bind IP        |Port |Packet enum
s = Server("172.17.21.47", 5001, PacketType)
s.open_server()
s.wait_for_connection()

try:
    while True:
        sleep(0.05)
        print("above the try")
        try:
            print("below the try")
            if TOP_RAMP_SENSOR.distance() <= 30:
                s.send_packet(PacketType.COMMAND1, b"15")
            if LOWER_RAMP_SENSOR.distance() <= 60:
                s.send_packet(PacketType.COMMAND1, b"17")
            print("bottommmmmm")
        except OSError as GO_PACK_GO:
            print("skdjflskdjflskjdflksd")
            counter += 1
        if joy.get_button_state(0) == True:
            while joy.get_button_state(0) == True:
                sleep(0.1)
            s.send_packet(PacketType.COMMAND1, b"0")
            print("the main BUTTTONNN")
        elif joy.get_button_state(2) == True:
            while joy.get_button_state(2) == True:
                sleep(0.1)
            s.send_packet(PacketType.COMMAND1, b"1")
        elif joy.get_button_state(1) == True:
            while joy.get_button_state(1) == True:
                sleep(0.1)
            s.send_packet(PacketType.COMMAND1, b"2")
        elif joy.get_button_state(3) == True:
            while joy.get_button_state(3) == True:
                sleep(0.1)
            s.send_packet(PacketType.COMMAND1, b"3")
        elif joy.get_button_state(4) == True:
            while joy.get_button_state(4) == True:
                sleep(0.1)
            s.send_packet(PacketType.COMMAND1, b"4")
        elif joy.get_button_state(9) == True:
            print("yooo")
            s.send_packet(PacketType.COMMAND1, b"7")
            s.send_packet(PacketType.COMMAND1, b"9")

finally:
    s.close_connection()
    s.close_server()