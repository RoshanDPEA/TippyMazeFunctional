import sys

sys.path.append("/home/pi/packages")
sys.path.append("/home/pi/packages/dpea-odrive")
sys.path.append("/home/pi/packages/RaspberryPiCommon/pidev")
sys.path.append("/home/pi/TippyMaze2.0")
sys.path.append("/home/pi/packages/slushengine")

from LidarSensor import LidarSensor
from time import sleep
from threading import Thread
from odrive_helpers import *
from Joystick import Joystick
from GumBallRelease import *
import enum
from dpea_p2p import Server
import spidev
from Slush.Devices import L6470Registers as LReg
from stepper import stepper
from Cyprus_Commands import Cyprus_Commands_RPi as cyprus

spi = spidev.SpiDev()
cyprus.initialize()
odrv = find_odrive()
ax0 = ODriveAxis(odrv.axis0, current_lim=15, vel_lim = 20)
ax1 = ODriveAxis(odrv.axis1, current_lim=15, vel_lim = 20)
# s0 = stepper(port=2, micro_steps=4, hold_current=2, run_current=26, accel_current=26, deaccel_current=26, steps_per_unit=200, speed=1)
ax0.axis.config.enable_watchdog = False
ax0.axis.error = 0
ax0.axis.config.watchdog_timeout = 0.5
ax1.axis.config.enable_watchdog = False
ax1.axis.error = 0
ax1.axis.config.watchdog_timeout = 0.5
joy = Joystick(0, False)
counter = 0
speed = 12
the_sleep = 0.1
joyvalue = None
top_sensor = False
bottom_sensor = False

TOP_RAMP_SENSOR = LidarSensor(port=5, threshold=70)
LOWER_RAMP_SENSOR = LidarSensor(port=3, threshold=70)
# GUMBALL_RELEASE = GumBallReleaseMotor(motor_port=3, sensor_port=4, sensor_threshold=35)
DPEA_MOTOR = DPEAMotor(motor_port=2)

class PacketType(enum.Enum):
    NULL = 0
    COMMAND1 = 1
    COMMAND2 = 2

#          |Server IP           |Port |Packet enum
s = Server("172.17.21.47", 5001, PacketType)

def run():
    Thread(target=sensor, daemon=True).start()
    print("\n\n", "started thread...", "\n\n")
    calibrate()
    print("\n\n", "done calibrating...", "\n\n")
    home()
    print("\n\n", "done homing...", "\n\n")

def calibrate():
    assert odrv.config.enable_brake_resistor is True, "CHECK FOR FAULTY BRAKE RESISTOR!"
    ax0.set_gains()
    ax1.set_gains()
    if not ax0.is_calibrated():
        print("\n\n", "calibrating axis0...", "\n\n")
        ax0.calibrate()
    if not ax1.is_calibrated():
        print("\n\n", "calibrating axis1...", "\n\n")
        ax1.calibrate()
    dump_errors(odrv)

def home():
    ax0.set_vel(-1)
    ax0.wait_for_motor_to_stop()
    ax0.set_home()
    ax0.set_pos_traj(9, 1, 4, 1)
    sleep(2)
    ax0.wait_for_motor_to_stop()
    ax1.set_vel(-1)
    ax1.wait_for_motor_to_stop()
    ax1.set_home()
    ax1.set_pos_traj(9, 1, 4, 1)
    sleep(2)
    ax1.wait_for_motor_to_stop()

# def gumball_release():
#     s0.start_relative_move(50)
#     while not top_sensor:
#         sleep(0.1)
#     print("\n\n", "gumball detected...", "\n\n")
#     s0.stop()
#     s0.free()

def is_next_move_valid_x():
    x_val = joy.get_axis("x")
    next_x = ax1.get_pos()
    if x_val > 0:
        next_x -= abs(joy.get_axis("x"))*(speed*the_sleep)*1.5
    elif x_val < 0:
        next_x += abs(joy.get_axis("x"))*(speed*the_sleep)*1.5
    return 2 < next_x < 16

def is_next_move_valid_y():
    y_val = joy.get_axis("y")
    next_y = ax0.get_pos()
    if y_val > 0:
        next_y += abs(joy.get_axis("y"))*(speed*the_sleep)*1.5
    elif y_val < 0:
        next_y -= abs(joy.get_axis("y"))*(speed*the_sleep)*1.5
    return 2 < next_y < 16

def joystick():
    global top_sensor, bottom_sensor
    ax0.axis.watchdog_feed()
    ax1.axis.watchdog_feed()
    ax0.axis.config.enable_watchdog = True
    ax1.axis.config.enable_watchdog = True
    while bottom_sensor == False:
        ax0.axis.watchdog_feed()
        ax1.axis.watchdog_feed()
        if is_next_move_valid_x():
            ax1.set_vel(joy.get_axis("x")*-speed)
        else:
            ax1.set_vel(0)
        if is_next_move_valid_y():
            ax0.set_vel(joy.get_axis("y")*speed)
        else:
            ax0.set_vel(0)
        sleep(0.1)
    ax0.axis.config.enable_watchdog = False
    ax1.axis.config.enable_watchdog = False
    ax0.idle()
    ax1.idle()
    sleep(0.5)
    top_sensor = False
    bottom_sensor = False
    DPEA_MOTOR.start_run()
    ax0.set_pos_traj(9, 1, 4, 1)
    ax0.wait_for_motor_to_stop()
    ax1.set_pos_traj(9, 1, 4, 1)
    ax1.wait_for_motor_to_stop()

def sensor():
    global counter, top_sensor, bottom_sensor
    s.open_server()
    s.wait_for_connection()
    print("\n\n", "connection successful...", "\n\n")

    while True:
        sleep(0.05)
        try:
            if TOP_RAMP_SENSOR.distance() <= 30:
                s.send_packet(PacketType.COMMAND1, b"15")
                top_sensor = True
            if LOWER_RAMP_SENSOR.distance() <= 60:
                s.send_packet(PacketType.COMMAND1, b"17")
                bottom_sensor = True
        except OSError as GO_PACK_GO:
            counter += 1
        if (cyprus.read_gpio() & 0b0001) == 0:
            sleep(0.1)
            if (cyprus.read_gpio() & 0b0001) == 0:
                s.send_packet(PacketType.COMMAND1, b"7")
                while (cyprus.read_gpio() & 0b0001) == 0:
                    sleep(0.1)
                s.send_packet(PacketType.COMMAND1, b"9")
        if joy.get_button_state(0) == True:
            while joy.get_button_state(0) == True:
                sleep(0.1)
            s.send_packet(PacketType.COMMAND1, b"0")
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

if __name__ == "__main__":
    try:
        run()
        while True:
            while top_sensor == False:
                sleep(0.1)
            joystick()
    finally:
        s.close_server()
        cyprus.close()
        ax0.idle()
        ax1.idle()
        print("\n\n", "COUNTER = ", counter,"\n\n")
