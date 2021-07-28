from rpiDevices.Devices.rpi import vppr

if __name__ == '__main__':
    # Initialise voltage proportional pressure regulator
    reg = vppr()

    # Set pressure in bar
    reg.set_P(0.1)
    reg.set_P(-1)

    # SHUTDOWN device
    reg.shutdown()