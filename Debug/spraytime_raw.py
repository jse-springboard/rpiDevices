# -*- coding: utf-8 -*-
"""
SPRAY TIME
----------

Created 2021-06-01
@author: Jordan Eriksen
"""
import numpy as np
import pandas as pd
import datetime
from time import sleep
import time
import os
import sys
import socket
import subprocess
import re
import board
import busio
# import adafruit_bno055
import adafruit_vl6180x

class tof:
    '''
    Class for handling of VL6180x time of lfight sensor with RPi
    ------------------------------------------------------------
    Optionally pass the I2C address of the sensor (default is 41)

    range = return distance to object in mm
    read_lux(gain) = returns lux from sensor according to gain specified:
    ALS_GAIN_1 = 1x 
    ALS_GAIN_1_25 = 1.25x
    ALS_GAIN_1_67 = 1.67x 
    ALS_GAIN_2_5 = 2.5x 
    ALS_GAIN_5 = 5x 
    ALS_GAIN_10 = 10x 
    ALS_GAIN_20 = 20x 
    ALS_GAIN_40 = 40x
    '''
    def __init__(self,address=41):
        self.address = address
        self.i2c = busio.I2C(board.SCL, board.SDA)
        # Create sensor instance.
        self.sensor = adafruit_vl6180x.VL6180X(self.i2c)
        # self.imu = adafruit_bno055.BNO055_I2C(i2c)
    
    def range(self,scale=1):
        '''
        Get the distance to object in mm*scale where scale has default of 1.
        '''
        return scale*self.sensor.range

# class imu:
#     '''
#     Class for the handling of the BNO055 9-DOF IMU with RPi
#     -------------------------------------------------------
#     '''

#     def __init__(self):
#         self.i2c = board.I2C()
#         self.sensor = adafruit_bno055.BNO055_I2C(i2c)
#         self.last_val = 0xFFFF

#     def temperature(self):
#         global last_val  # pylint: disable=global-statement
#         result = self.sensor.temperature

#         if abs(result - self.last_val) == 128:
#             result = self.sensor.temperature
#             if abs(result - self.last_val) == 128:
#                 return 0b00111111 & result

#         last_val = result

#         return result

def setup():
    '''
    Setup spray-time test
    ---------------------

    OUTPUTS AS DICT:
    tof_cartridge: tof instance
    test_date
    test_time
    data_out
    identifier
    counter
    '''
    # Initialise instances of adc and vppr classes
    print('')
    print('\033[1mInitialising time of flight sensor\033[0m')
    tof_cartridge = tof()

    # Set filename for all repeats to show the start time of the first test to enble tests to be matched later
    test_date = datetime.datetime.now().date()
    test_time = datetime.datetime.now().strftime("%H-%M")
    
    '''
    ## Define test ID
    Test ID either taken directly from the arguments passed from the command line, or if not present, ask operator for one
    '''
    try:
        identifier = str(sys.argv[1])
    except IndexError:
        identifier = str(input('Enter test identifier: '))
    print(f'----------------------------------- \n')

    return {'tof_cartridge':tof_cartridge, 'test_date':test_date, 'test_time':test_time, 'identifier':identifier, 'counter':0}
    
def _prime_device():
    '''
    Handle priming of each test.

    After running program, ask user for readiness.

    3-2-1-FIRE countdown. Return 1 on user input.
    '''
    print(f'\n SPRAY TIME TEST \n ---------------')
    print(f'Prime the device VERTICALLY, then CTRL-C ... \n')

    while True:
        try:
            print('***CTRL-C WHEN READY***',end='\r')
            time.sleep(0.5)
            print('   CTRL-C WHEN READY   ',end='\r')
            time.sleep(0.5)
            print('***CTRL-C WHEN READY***',end='\r')
            time.sleep(0.5)
            print('   CTRL-C WHEN READY   ',end='\r')
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
    
    return 1


def run_test(setup_dict, df):
    '''
    Run test once primed using setup parameters specified
    '''
    primed = _prime_device()

    _tof = setup_dict['tof_cartridge']

    if primed == 1:
        armed = 1
        print(f'\r Begin recording (armed = {armed})\n')

        t0 = time.time()
        while armed == 1:
                _time = [time.time() - t0]
                _dist = [_tof.range()]
                    
                #_grad = np.polyfit(_time,_dist,1)
                line = pd.DataFrame({'Time':_time,'Distance':_dist}, 
                                    columns=['Time','Distance'], 
                                    index=[0])

                df = df.append(line, ignore_index=True)
                
        setup_dict['counter'] += 1

        return setup_dict, df
    else:
        print(f'NOT primed')
        raise ValueError


if __name__ == '__main__':
    '''
    Main program to run
    '''

    setup_dict = setup()
    count = 0

    while True:
        count += 1

        # Setup empty data frame for results
        data_out = pd.DataFrame(columns=['Time','Distance','delta'])

        setup_dict, data_out = run_test(setup_dict, data_out)
        
        if count == 1:
            os.system(f"mkdir /home/pi/Projects/Nebula/Data_{socket.gethostname()}/{setup_dict['test_date']}_{setup_dict['test_time']}_SPRAY_{setup_dict['identifier']}")
        filename_csv = f"/home/pi/Projects/Nebula/Data_{socket.gethostname()}/{setup_dict['test_date']}_{setup_dict['test_time']}_SPRAY_{setup_dict['identifier']}/{setup_dict['test_date']}_{setup_dict['test_time']}_SPRAY_{setup_dict['identifier']}_{count}.csv"
        filename_pkl = f"/home/pi/Projects/Nebula/Data_{socket.gethostname()}/{setup_dict['test_date']}_{setup_dict['test_time']}_SPRAY_{setup_dict['identifier']}/{setup_dict['test_date']}_{setup_dict['test_time']}_SPRAY_{setup_dict['identifier']}_{count}.pkl"
        
        data_out.to_pickle(filename_pkl)
        data_out.to_csv(filename_csv)
        
        continue_ = input(f'\r Run test again? [Y/n]:  ')

        if (continue_ == 'Y') | (continue_ == 'y'):
            continue
        else:
            break

    # Save output to git repository
    os.system(f"cd Data_{socket.gethostname()} && git add -A && git commit -m '{setup_dict['identifier']}' && cd ..")