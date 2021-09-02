from rpiDevices.rpi import imu
import sys

def main():
    '''
    Main executable method for example of the 9-DOF IMU by Adafruit
    ---------------------------------------------------------------
    '''

    i = imu()
    i.setMode

    if not sys.argv[1]:
        i.print(sys.argv[1])
