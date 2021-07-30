'''
Author: Jordan Eriksen

Classes for running test components
-----------------------------------

ADC24 - Picoscope ADC 20/24 and terminal board
TC08 - Picoscope TC08 thermocouple reader
VPPR - Voltage proportional pressure regulator
TOF - Time of flight sensor VL6180x
IMU - 9-dof Inertial Measurement Unit
PCE_Loadcell - PCE handheld loadcell
'''

import numpy as np
import pandas as pd
import math as m
import datetime, time, os, sys, subprocess, serial, re, socket, ctypes
from time import sleep

import board
import busio
# import Adafruit_BNO055 as adafruit_bno055
import adafruit_vl6180x
from picosdk.usbtc08 import usbtc08
from picosdk.picohrdl import picohrdl as hrdl
from picosdk.functions import assert_pico2000_ok
import RPi.GPIO as gpio

class adc24:
    '''
    Class for the handling of a Picolog ADC20/24 using PicoSDK
    ----------------------------------------------------------

    Requires a powered USB hub to power the ADC.
    SUPPORTS MULTIPLE CHANNELS
    - 'channel' argument requires entries as a dictionary containing the channel number as the key and the ADC value conversion coefficients as the values (in a list).
    '''

    def __init__(self, channel={2:[0,21],15:[-0.105,2.5*32]}):
        # Create chandle and status ready for use
        self.chandle = ctypes.c_int16()
        self.status = {}
        self.numsamples = 1
        self.channel = list(channel.keys()) # Set input channel here
        self.coefficients = channel
        self.numchannels = len(self.channel)

        self._startup()

    def _startup(self):
        # Open unit
        self.status["openUnit"] = hrdl.HRDLOpenUnit()
        assert_pico2000_ok(self.status["openUnit"])
            
        self.chandle=self.status["openUnit"]

        # Set mains noise rejection
        # Reject 50 Hz mains noise
        self.status["mainsRejection"] = hrdl.HRDLSetMains(self.chandle, 0)
        assert_pico2000_ok(self.status["mainsRejection"])

        # Open ADC channel
        vrange = hrdl.HRDL_VOLTAGERANGE["HRDL_2500_MV"]

        # Define dictionaries of min and max ADC values with channel number as a key
        self.minAdc = {}
        self.maxAdc = {}

        for i in self.channel:
            self.status[f'activeChannel_{i}'] = hrdl.HRDLSetAnalogInChannel(self.chandle, i, 1, vrange, 1)
            assert_pico2000_ok(self.status[f'activeChannel_{i}'])

            # Get min and max adc values
            # Setup pointers
            self.minAdc[i] = ctypes.c_int32(0)
            self.maxAdc[i] = ctypes.c_int32(0)

            # Get min/max outputs for pressure transducer
            self.status[f'minMaxAdcCounts_{i}'] = hrdl.HRDLGetMinMaxAdcCounts(self.chandle, ctypes.byref(self.minAdc[i]), ctypes.byref(self.maxAdc[i]), i)
            assert_pico2000_ok(self.status[f'minMaxAdcCounts_{i}'])
        
        # Set sampling time interval
        conversionTime = hrdl.HRDL_CONVERSIONTIME["HRDL_60MS"]
        self.status["samplingInterval"] = hrdl.HRDLSetInterval(self.chandle, 121, conversionTime)
        assert_pico2000_ok(self.status["samplingInterval"])

    def _get_data(self,nums_=4):
        '''
        Function using ADC method to extract data
        '''
        # Start sampling with BM_BLOCK (0) method
        self.status["collectingSamples"] = hrdl.HRDLRun(self.chandle, nums_, 0)
        assert_pico2000_ok(self.status["collectingSamples"])

        # While loop to pause program while data is collected
        while hrdl.HRDLReady(self.chandle) == 0: 
            sleep(0.05)

        '''
        ## Get values
        Setup pointer to output location with correct size
        Give pointers to ADC method to save output data to
        '''
        overflow = ctypes.c_int16(0)
        values = (ctypes.c_int32 * (nums_ * self.numchannels))()
        times = (ctypes.c_int32 * (nums_ * self.numchannels))()
        self.status["getTimesAndValues"] = hrdl.HRDLGetTimesAndValues(self.chandle, ctypes.byref(times), ctypes.byref(values), ctypes.byref(overflow), nums_)
        assert_pico2000_ok(self.status["getTimesAndValues"])

        return overflow, values, times

    def all_out(self,nums=4):
        '''
        Collect a data point for all inputs
        -----------------------------------

        Use 'coeffs' to specify the coefficients used to convert the raw ADC signal ( 0 - self.maxAdc.value )
        'coeffs' is defined as a dictionary with the key being the channel number and the value being a list of coefficients in increasing order i.e. [x0, x1] where y = x1*x + x0
        '''

        # Return arrays of time, pressure and flow rate
        overflow, values, times = self._get_data(nums_=nums)
        
        '''
        ## Convert the output data from a ctype array to a real value. 
        ADC outputs data from input channels alternately. Slicing [start:end:interval] used to pick every other data point.
        Conversion equation turns raw ADC output into a physical value
        '''
        output = {}
        times_out = {}

        for n in range(self.numchannels):
            ch = self.channel[n]
            x0 = self.coefficients[ch][0]
            x1 = self.coefficients[ch][1]

            output[self.channel[n]] = np.around(((np.average(np.ctypeslib.as_array(values[n::2])) * x1/self.maxAdc[ch].value) + x0), decimals=4)
            times_out[self.channel[n]] = np.around(np.average(np.ctypeslib.as_array(times[n::2])/1000),decimals=4)

        return output, times_out

    def shutdown(self):
        '''
        Safely shutdown the ADC using the HRDLCloseUnit command 
        '''
        # Close unit
        self.status["closeUnit"] = hrdl.HRDLCloseUnit(self.chandle)
        assert_pico2000_ok(self.status["closeUnit"])

class vppr:
    '''
    Class for the handling of a voltage proportional pressure regulator using a RPi
    -------------------------------------------------------------------------------

    Requires a DAC to convert PWM signal to a voltage.
    Connected DAC has required output voltage (10V) and can run off a 15V supply.

    Check that required supporting files are in the correct place:
        - Requires a fit_data.npy file to manage the offset between demand pressure
          and actual output pressure
        - .//Setup//fit_data.npy
    '''
    def __init__(self,minout=0.0,maxout=10, pwm_freq=1.2e3, pwm_pin=12, mode='hw'):
        self.minout = minout # Minimum output pressure set by VPPR
        self.maxout = maxout # Maximum output pressure set by VPPR
        self.pwm_freq = pwm_freq # 
        self.pwm_pin = pwm_pin
        self.mode = mode
        
        # Load a fit curve to manage the offset between demand and actual output pressure
        self.fitdata = np.load('./Devices/fit_data.npy',allow_pickle=True)
        self.fit_pressure = np.polynomial.Polynomial.fit(self.fitdata[0,:],self.fitdata[1,:],9)

        # Define mode of operation. Default should be hardware ('hw').
        if self.mode == 'sw':
            self._startup_SW()
        else:
            self._startup()

    def _startup_SW(self):
        '''
        Function starts the software handling of the PWM signal using RPi.GPIO module
        '''
        # Set pin numbering format
        gpio.setmode(gpio.BCM)

        # Setup pwm pin as output
        gpio.setup(self.pwm_pin,gpio.OUT)

        # Begin PWM
        self.pwm = gpio.PWM(self.pwm_pin,self.pwm_freq)
        self.pwm.start(0)

    def _startup(self):
        '''
        Begin pigpio deamon to handle hardware PWM on the RPi. Output is supressed to reduce clutter.
        '''
        # Will raise error if daemon already running, error is supressed using stderr=subprocess.DEVNULL
        subprocess.run(f'sudo pigpiod', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) 
        sleep(0.2)

        # Start PWM and set to 0
        subprocess.run(f'pigs hp {self.pwm_pin} {int(self.pwm_freq)} {0}', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def set_P(self,pressure=-1):
        '''
        Set the pressure of the regulator by mapping pwm duty -> DAC output -> pressure;
        Voltage = PWM% x 0.1;
        Pressure = Voltage * (P_max - P_min)/10 + P_min;
        
        Hence PWM = ((Pressure - P_min)/(P_max - P_min))*100
        '''
        
        # Output pressure must be greater than the minimum output of the VPPR
        if pressure < self.minout:
            # Use pigs to change hardware PWM signal to 0 if pressure is too low
            os.system(f'pigs hp {self.pwm_pin} {int(self.pwm_freq)} {int(0)}')

        else:
            # To reduce strain on the system, each pressure is reached by ramping up to the required pressure over 10 steps.
            # Infer last demand pressure from the current PWM output using pigs gdc <pwm_pin> (GET DUTY CYCLE)
            proc = subprocess.run(f'pigs gdc {self.pwm_pin}', shell=True, encoding='utf-8', stdout=subprocess.PIPE)

            # Account for non-linearity of VPPR by using a lookup to determine demand pressure required to set actual pressure
            # See AIV-00019-revA for details on this
            pressure = self.fit_pressure(pressure)

            # Calculate last demand p using the PWM signal
            # Equation is the inverse of that at the top of the function
            pressure_last = (float(proc.stdout)/1000000)*(self.maxout - self.minout) + self.minout
            
            #Setup pressure ramp
            pressure_ramp = np.linspace(pressure_last,pressure,10)

            # Cunduct pressure ramp using the pigs hp method to set PWM output
            for press in pressure_ramp:
                pwm_set = ((press - self.minout)/(self.maxout- self.minout))*100
                if round(pwm_set) > 100:
                    pwm_set = 100
                else:
                    pass
                os.system(f'pigs hp {self.pwm_pin} {int(self.pwm_freq)} {int(pwm_set*10000)}')
        
    def _set_P_SW(self,pressure=-1):
        '''
        NOT USED - CHECK FUNCTIONALITY BEFORE CALLING

        Uses the software PWM in RPI.GPIO instead of hardware. Less accurate and slower.

        Set the pressure of the regulator by mapping pwm duty -> DAC output -> pressure;
        Voltage = PWM% x 0.1;
        Pressure = Voltage * (P_max - P_min)/10 + P_min;
        
        Hence PWM = ((Pressure - P_min)/(P_max - P_min))*100
        '''

        if pressure < self.minout:
            self.pwm.ChangeDutyCycle(0)
        else:
            # pressure = self.fitdata(pressure)
            pwm_set = ((pressure - self.minout)/(self.maxout- self.minout))*100
            self.pwm.ChangeDutyCycle(pwm_set)

    def shutdown(self):
        '''
        Safely shutdown the VPPR
        '''
        if self.mode == 'hw': # If using hardware PWM, set dutycycle to 0 to shutdown PWM
            self.set_P() # Calling set_P without an argument sets the dutycycle to 0
        else: # If using software PWM, stop PWM and cleanup GPIO pins
            self.set_P()
            self.pwm.stop()
            gpio.cleanup()

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

#     UPDATES REQUIRED:
#     - Support for IMU functions as outputs
#     - KARMAN filter for inertial navigation?
#     '''

#     def __init__(self):
#         self.i2c = board.I2C()
#         self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
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

class tc08:
    '''
    Class for the handling of a Picolog TC08 using PicoSDK
    ------------------------------------------------------

    Requires a powered USB hub to power the TC08 unit.

    'channel' argument can take any number of inputs but MUST BE A LIST.

    temp, cold_junction = tc8.get_temp()
    temp = {channel: temperature ... }

    Use tc08.shutdown() after use!
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

    def get_temp(self,cold_out='no'):
        '''
        Collect a data point for all inputs

        Outputs a dictionary of channel numbers and corresponding temperature in Celcius.
        '''
        # Return arrays of temperature for all channels
        temp = self._get_data_single()

        cold_junction = temp[0]
        temps = {int(ch): temp[int(ch)] for ch in self.channel}

        if str(cold_out) == 'no':
            return temps
        else:
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
    --------------------------------------------------
    POWERED USB CONNECTION TO RPI REQUIRED
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
