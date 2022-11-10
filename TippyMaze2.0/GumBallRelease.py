import sys

sys.path.append("/home/pi/packages")
sys.path.append("/home/pi/packages/dpea-odrive")
sys.path.append("/home/pi/packages/RaspberryPiCommon/pidev")
sys.path.append("/home/pi/TippyMaze2.0")
sys.path.append("/home/pi/packages/slushengine")

from time import sleep
from Slush.Devices import L6470Registers as LReg
from stepper import stepper

GUMBALL_MOTOR_SETTINGS = {
    'hold_current': 2,
    'run_current': 26,
    'acc_current': 26,
    'dec_current': 26,
    'max_speed': 300,
    'min_speed': 0,
    'micro_steps': 4,
    'threshold_speed': 10000,
    'over_current': 3000,
    'stall_current': 3125,
    'accel': 0xffe,
    'decel': 0xffe,
    'slope': [0x0F05, 0x2F, 0x5A,0x5A]
}

NEMA_17 = {
    'hold_current': 8,
    'run_current': 10,
    'acc_current': 10,
    'dec_current': 10,
    'max_speed': 525,
    'min_speed': 0,
    'micro_steps': 32,
    'threshold_speed': 1000,
    'over_current': 2000,
    'stall_current': 2187.5,
    'accel': 0x50,
    'decel': 0x10,
    'low_speed_opt': False,
    'slope': [0x562, 0x010, 0x01F, 0x01F]
    }


class GumBallReleaseMotor:
    def __init__(self, motor_port, sensor_port, sensor_threshold, timeout=20):
        self.motor = stepper(port=motor_port, speed=GUMBALL_MOTOR_SETTINGS['max_speed'])
        self.motor.setCurrent(hold=GUMBALL_MOTOR_SETTINGS['hold_current'], run=GUMBALL_MOTOR_SETTINGS['run_current'],
                              acc=GUMBALL_MOTOR_SETTINGS['acc_current'], dec=GUMBALL_MOTOR_SETTINGS['dec_current'])
        self.motor.setMaxSpeed(GUMBALL_MOTOR_SETTINGS['max_speed'])
        self.motor.setMinSpeed(GUMBALL_MOTOR_SETTINGS['min_speed'])
        self.motor.setMicroSteps(GUMBALL_MOTOR_SETTINGS['micro_steps'])
        self.motor.setThresholdSpeed(GUMBALL_MOTOR_SETTINGS['threshold_speed'])
        self.motor.setOverCurrent(GUMBALL_MOTOR_SETTINGS['over_current'])
        self.motor.setStallCurrent(GUMBALL_MOTOR_SETTINGS['stall_current'])
        self.motor.setAccel(GUMBALL_MOTOR_SETTINGS['accel'])
        self.motor.setDecel(GUMBALL_MOTOR_SETTINGS['decel'])
        slope = GUMBALL_MOTOR_SETTINGS['slope']
        self.motor.setLowSpeedOpt(False)
        self.motor.setParam(LReg.CONFIG, 0x3688)

    def release_gumball(self):
        """
        Release a gumball by running the motor until the lidar sensor detects a gumball.
        Warning this method is blocking.
        :return: None
        """
        #self.released = False
        self.motor.run(dir=1, spd=GUMBALL_MOTOR_SETTINGS['max_speed'])

        sleep(15)
        # while not TOP_RAMP_SENSOR.distance() <= 30:
        #     sleep(0.05)

        # while not self.sensor.detected:
        #     sleep(.05)
        #     self.sensor.refresh_last_read()

        print("\n\n", "gumball detected...", "\n\n")
        self.motor.hard_stop()

        # self.sensor.reset()  # reset the sensor for next release
        #self.released = True

    def free(self):
        """
        Free the attached motor
        :return: None
        """
        self.motor.free()

class DPEAMotor:
    def __init__(self, motor_port):
        """
        :param motor_port: port the DPEA gears motor goes on
        """
        self.motor = stepper(port=motor_port, speed=NEMA_17['max_speed'])
        self.setup_motor()

    def setup_motor(self):
        # self.motor.setCurrent(hold=NEMA_17['hold_current'],run=NEMA_17['run_current'],
        #                       acc=NEMA_17['acc_current'],dec=NEMA_17['dec_current'])
        self.motor.setMaxSpeed(NEMA_17['max_speed'])
        self.motor.setMinSpeed(NEMA_17['min_speed'])
        self.motor.setMicroSteps(NEMA_17['micro_steps'])
        self.motor.setThresholdSpeed(NEMA_17['threshold_speed'])
        self.motor.setOverCurrent(NEMA_17['over_current'])
        self.motor.setStallCurrent(NEMA_17['stall_current'])
        self.motor.setAccel(NEMA_17['accel'])
        self.motor.setDecel(NEMA_17['decel'])

    def start_run(self):
        self.motor.run(dir=1, spd=NEMA_17['max_speed'])
        sleep(2)
        print("\n\n", "motor stopped...", "\n\n")
        self.motor.softStop()