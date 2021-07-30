from rpiDevices.rpi import tc08

if __name__ == '__main__':
    # Initiate an instance of the tc08 class to startup the unit
    # Specify the channels used by the instance.

    channels = [1,3,5,7]

    tc = tc08(channel=channels)

    # tc08.get_temp() returns both the temperatures on the channels specified (as a dict) and the temperature of the cold junction.
    output_temp = tc.get_temp()

    for ch in channels:
        print(f'Ch {ch} temperature = {output_temp[ch]:.2f}')

    # Shutdown unit
    tc.shutdown()