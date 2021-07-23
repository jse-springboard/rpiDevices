# -*- coding: utf-8 -*-
"""
LEAKRIG
-------

Created 2021-05-27
@author: Jordan Eriksen
"""
import ctypes
import numpy as np
import pandas as pd
import datetime
from time import sleep
import time
import os
import sys
import subprocess
import socket
import serial
import re

from picosdk.usbtc08 import usbtc08
from picosdk.functions import assert_pico2000_ok

class tc08:
    '''
    Class for the handling of a Picolog TC08 using PicoSDK
    ------------------------------------------------------

    Requires a powered USB hub to power the TC08 unit.
    '''

    def __init__(self, channel=[1,2]):
        # Create chandle and status ready for use
        self.chandle = ctypes.c_int16()
        self.status = {}
        self.numsamples = 1
        self.channel = channel # Set input channel here

        self._startup()

    def _startup(self):
        # open unit
        self.status["open_unit"] = usbtc08.usb_tc08_open_unit()
        assert_pico2000_ok(self.status["open_unit"])
        self.chandle = self.status["open_unit"]

        # set mains rejection to 50 Hz
        self.status["set_mains"] = usbtc08.usb_tc08_set_mains(self.chandle,0)
        assert_pico2000_ok(self.status["set_mains"])

        # set up channel
        # therocouples types and int8 equivalent
        # B=66 , E=69 , J=74 , K=75 , N=78 , R=82 , S=83 , T=84 , ' '=32 , X=88 
        typeK = ctypes.c_int8(75)

        for ch in self.channel:
            ch = int(ch)
            self.status[f"set_channel_{ch}"] = usbtc08.usb_tc08_set_channel(self.chandle, ch, typeK)
            assert_pico2000_ok(self.status[f"set_channel_{ch}"])


        # get minimum sampling interval in ms
        self.status["get_minimum_interval_ms"] = usbtc08.usb_tc08_get_minimum_interval_ms(self.chandle)
        assert_pico2000_ok(self.status["get_minimum_interval_ms"])

    def _get_data_single(self):
        '''
        Function using ADC method to extract data
        '''
        # Get single temperature reading
        temp = (ctypes.c_float * 9)()
        overflow = ctypes.c_int16(0)
        units = usbtc08.USBTC08_UNITS["USBTC08_UNITS_CENTIGRADE"]
        self.status["get_single"] = usbtc08.usb_tc08_get_single(self.chandle,ctypes.byref(temp), ctypes.byref(overflow), units)
        assert_pico2000_ok(self.status["get_single"])

        return temp

    def get_temp(self):
        '''
        Collect a data point for all inputs
        '''
        # Return arrays of temperature for all channels
        temp = self._get_data_single()

        cold_junction = temp[0]
        temps = {int(ch): temp[int(ch)] for ch in self.channel}

        return temps, cold_junction

    def shutdown(self):
        '''
        Safely shutdown the TC08 using the close_unit command 
        '''
        # close unit
        self.status["close_unit"] = usbtc08.usb_tc08_close_unit(self.chandle)
        assert_pico2000_ok(self.status["close_unit"])

class PCE_Loadcell:
    '''
    Class for handling of PCE_Loadcell via serial port
    '''
    def __init__(self,max_load=500,increment=0.1,baud=9600,port='/dev/ttyUSB0'):
        self.max_load = max_load
        self.increment = increment
        self.baud = baud
        self.port = port

        self.serial = serial.Serial(
            port=self.port,
            baudrate = self.baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

    def get_force(self,samples=10,mode=1):
        '''
        Read cell. Modes defined as follows:
         - 0: (Default) Single reading of specified number of samples
         - 1: Single reading of a single sample
        '''
        self.serial = serial.Serial(
            port=self.port,
            baudrate = self.baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        _out = []

        if mode == 0:
            for i in range(samples):
                while True:
                    try:
                        line = str(self.serial.readline(), 'UTF-8')
                        _out[i] = float(re.split(r'\.',line)[0]) + float(re.match(r'^\d',re.split(r'\.',line)[1]).group(0))/10
                        break
                    except (ValueError,AttributeError):
                        pass
                _out = np.mean(_out)
        elif mode == 1:
            while True:
                    try:
                        line = str(self.serial.readline(), 'UTF-8')
                        # print(line)
                        _out = float(re.split(r'\.',line)[0]) + float(re.match(r'^\d',re.split(r'\.',line)[1]).group(0))/10
                        # print(_out)
                        return _out
                    except (ValueError,AttributeError,IndexError):
                        pass
        
        return _out

def get_time():
    '''
    Function handling the collection of the time of test from the operator
    '''
    try:
        time = input(f'Time period for leakage test (eg "20s", "5m", "1h", "1d"): ')
        try:
            seconds = float(time[re.search(r'\d+s',time).start():re.search(r'\d+s',time).end()-1])
            # seconds = float(re.split(r's',re.match(r'\d+s',time).group(0))[0])
        except (TypeError,AttributeError):
            seconds = 0

        try:
            minutes = float(time[re.search(r'\d+m',time).start():re.search(r'\d+m',time).end()-1])
            # minutes = float(re.split(r'm',re.match(r'\d+m',time).group(0))[0])
        except (TypeError,AttributeError):
            minutes = 0

        try:
            hours = float(time[re.search(r'\d+h',time).start():re.search(r'\d+h',time).end()-1])
            # hours = float(re.split(r'h',re.match(r'\d+h',time).group(0))[0])
        except (TypeError,AttributeError):
            hours = 0

        try:
            days = float(time[re.search(r'\d+d',time).start():re.search(r'\d+d',time).end()-1])
            # days = float(re.split(r'd',re.match(r'\d+d',time).group(0))[0])
        except (TypeError,AttributeError):
            days = 0

    except TypeError:
        print('Incorrect time passed as input. See examples given above.')
        return 0
    
    print(f'Leak test running for {seconds:.1f}s {minutes:.1f}m {hours:.1f}h {days:.1f}d')
    print(f'\n------------------------------------------- \n')
    return [seconds,minutes,hours,days]


if __name__ == '__main__':
    # Initialise instances of tc08 and loadcell classes
    print('')
    print('\033[1mInitialising loadcell and thermocouple\033[0m\n')
    loadcell = PCE_Loadcell()
    tc = tc08()

    test_date = datetime.datetime.now().date()
    test_clock = datetime.datetime.now().strftime("%H-%M")
    
    try:
        identifier = str(sys.argv[1])
    except IndexError:
        identifier = str(input('Enter test identifier: '))
    print(f'------------------------------------------- \n')

    while True:
        try:
            test_time = get_time()
            if test_time == 0:
                raise ValueError
            else:
                break
        except ValueError:
            pass
        except KeyboardInterrupt:
            raise ValueError('User ended program.')

    test_seconds = test_time[0] + 60*test_time[1] + 3600*test_time[2] + 24*3600*test_time[3]

    data = pd.DataFrame(columns=['Time','Force','Temperature'])

    filename = f'{test_date}_{test_clock}_LEAKAGE_{test_time[0]}s_{test_time[1]}m_{test_time[2]}h_{test_time[3]}d_{identifier}'
    
    t0 = time.time()
    elapsed_time = 0
    try:
        while elapsed_time < test_seconds:
            current_force = loadcell.get_force(mode=1)
            temps, cold_junction = tc.get_temp()

            try:
                if current_force < 10:
                    continue
            except IndexError:
                pass

            print(f'({elapsed_time:.1f}/{test_seconds:.1f}) Force = {current_force:.1f}N    Temperature = {temps[1]:.1f}      ',end='\r')

            data = data.append({'Time':elapsed_time,'Force':current_force,'Temperature':temps[1]}, ignore_index=True)

            time.sleep(0.9)
            elapsed_time = time.time() - t0
        raise KeyboardInterrupt    
    except KeyboardInterrupt:
        tc.shutdown()
        print(f'------------------------------------------- \n')

        print(f"Minimum force: {data['Force'].min():.1f}  at time: {data['Time'].iloc[data['Force'].idxmin()]:.1f}")
        print(f"Maximum force: {data['Force'].max():.1f}  at time: {data['Time'].iloc[data['Force'].idxmax()]:.1f}")
        print(f'\n------------------------------------------- \n')

        # Make folder for test results
        subprocess.run(f'mkdir ./Data_{socket.gethostname()}/{test_date}_LEAKAGE_{identifier}', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        fileloc=f'.//Data_{socket.gethostname()}//{test_date}_LEAKAGE_{identifier}'

        data.to_csv(f'{fileloc}//{filename}.csv',index=False)
        data.to_pickle(f'{fileloc}//{filename}.pkl')

        # Save output to git repository
        os.system(f'cd Data_{socket.gethostname()} && git add -A && git commit -m "{identifier}" && cd ..')
