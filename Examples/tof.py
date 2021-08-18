from rpiDevices.rpi import tof

if __name__ == '__main__':
    # Initialise TOF sensor
    sensor = tof()
    
    # Read distance
    distance = sensor.range()