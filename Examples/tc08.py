from numpy.lib import utils
from rpiDevices.Devices.rpi import tc08

if __name__ == '__main__':
    # Initiate an instance of the tc08 class to startup the unit
    # Specify the channels used by the instance.

    tc = tc08(channel=[1,2,3])

    # tc08.get_temp() returns both the temperatures on the channels specified (as a dict) and the temperature of the cold junction.
    output_temp = tc.get_temp()

    channel1_temp = output_temp[1]
    channel2_temp = output_temp[2]
    channel3_temp = output_temp[3]

    # Shutdown unit
    tc.shutdown()