'''
Author: Jordan Eriksen

Package of classes for running sensors and actuators with RPi
-------------------------------------------------------------

[adc24]\t-> Picoscope ADC 20/24 and terminal board
\t      -> Reference: https://www.picotech.com/download/manuals/adc20-adc24-high-resolution-data-logger-users-guide.pdf
    
[tc08]\t-> Picoscope TC08 thermocouple reader
[vppr]\t-> Voltage proportional pressure regulator
[tof]\t-> Time of flight sensor VL6180x
[imu]\t-> 9-dof Inertial Measurement Unit
[pce]\t-> PCE handheld loadcell

OTHER COMMANDS:
step(PR,ADC,pressure,testT,sampleT)
hold(PR,ADC,pressure,testT,sampleT)
impulse(PR,ADC,pressure,testT)

PR,ADC -> Instances of the pressure regulator and adc classes (already defined)
pressure -> Demand pressure in bar
testT -> Time to run test for in seconds
sampleT -> Approximate sample period
'''

import numpy as np
import pandas as pd
import os, subprocess, serial, re, ctypes
from time import sleep,time

import board
import busio
import adafruit_bno055
import adafruit_vl6180x
from picosdk.usbtc08 import usbtc08
from picosdk.picohrdl import picohrdl as hrdl
from picosdk.functions import assert_pico2000_ok
from picosdk.errors import PicoSDKCtypesError as perr
import RPi.GPIO as gpio
import rpiDevices

class adc24:
    '''
    Class for the handling of a Picolog ADC20/24 using PicoSDK
    ----------------------------------------------------------

    Requires a powered USB hub to power the ADC.
    SUPPORTS MULTIPLE CHANNELS
    - 'channel' argument requires entries as a dictionary containing the channel number as the key and the ADC value conversion coefficients as the values (in a list).

    Use channel.value() -> 'coeffs' to specify the coefficients used to convert the raw ADC signal ( 0 -> self.maxAdc.value )
        'coeffs' is a list of coefficients in increasing order i.e. [x0, x1] where y = x1*x + x0

    ## vrange:
    Use vrange as dict to specify voltage range for each channel. No entry for a declared channel will default to HRDL_2500_MV
    eg -> vrange = {1:0, 2:1} puts channel 1 in range +/-2500MV and channel 2 in range +/-1250MV.

    HRDL_2500_MV (0) ??2500 mV ADC-20 and ADC-24 \n
    HRDL_1250_MV (1) ??1250 mV ADC-20 and ADC-24 \n
    HRDL_625_MV (2) ??625 mV ADC-24 only \n
    HRDL_313_MV (3) ??312.5 mV ADC-24 only \n
    HRDL_156_MV (4) ??156.25 mV ADC-24 only \n
    HRDL_78_MV (5) ??78.125 mV ADC-24 only \n
    HRDL_39_MV (6) ??39.0625 mV ADC-24 only \n

    '''

    def __init__(self, channel={2:[0,21],15:[-0.105,2.5*32]},vrange={},chandle='None',buffer_size=5):
        # Create chandle and status ready for use
        # If channel is started using the handler, use chandle that has already been assigned.
        self.status = {}
        self.buffer_size = buffer_size

        try:
            if channel == 'nebula':
                channel={2:[0,21],15:[-0.105,2.5*32]}
            elif type(channel) == dict:
                pass
        except TypeError:
            pass

        self.channel = list(channel.keys()) # Set input channel here
        self.coefficients = channel
        self.numchannels = len(self.channel)
        self.streaming = 0
        self.vrange=vrange
        self.active = False
        self.vrOptions = {
            0:2.5,
            1:1.25,
            2:0.625,
            3:0.3125,
            4:0.15625,
            5:0.078125,
            6:0.0390625
        }

        # Set output pointers and arrays to save time
        self.overflow = ctypes.c_int16(0)
        self.values = (ctypes.c_int32 * (self.buffer_size * self.numchannels))()
        self.times = (ctypes.c_int32 * (self.buffer_size * self.numchannels))()
        
        if chandle=='None':
            self.chandle = ctypes.c_int16()
        else:
            self.chandle = chandle
        
        self._startup()

    def _startup(self):
        '''
        Setup device
        ------------

        Initially check to see if an existing chandle was passed.
        '''
        try:
            # Open unit
            self.status["openUnit"] = hrdl.HRDLOpenUnit()
            assert_pico2000_ok(self.status["openUnit"])
            self.chandle=self.status["openUnit"]
            self.active = True
        except perr as a:
            print(f'#################################################################################')
            print(f'ERROR: Unable to connect to the device. Ensure device is connected and try again.')
            print(f'#################################################################################\n')
            raise a
            
        # Set mains noise rejection
        # Reject 50 Hz mains noise by passing 0 as argument (<>0 for 60 Hz)
        self.status["mainsRejection"] = hrdl.HRDLSetMains(self.chandle, 0)
        assert_pico2000_ok(self.status["mainsRejection"])

        # Setup channels
        self._channelStartup(self.channel)
        
        # Set sampling time interval
        conversionTime = hrdl.HRDL_CONVERSIONTIME["HRDL_60MS"]
        self.sampleInterval_ms = self.numchannels*60 + 1
        self.status["samplingInterval"] = hrdl.HRDLSetInterval(self.chandle, self.sampleInterval_ms, conversionTime)
        assert_pico2000_ok(self.status["samplingInterval"])

    def _channelStartup(self,channel):
        '''
        Assign channels to the device
        -----------------------------
        
        Centralise the setting of new channels. To be called during startup and after assignment of new channels.\n

        channel:
        Either a list of channel numbers or a dictionary with channel numbers as keys and with coefficients as values. 
        '''
        # Define dictionaries of min and max ADC values with channel number as a key
        self.minAdc = {}
        self.maxAdc = {}

        if not channel:
            print(f'No channels assigned. Used addCh() to add channels to this device.')
            pass
        else:
            for i in channel:
                # Set voltage range
                try:
                    if i in self.vrange:
                        vrange = self.vrange[i]
                    else:
                        self.vrange[i] = 0
                        vrange = hrdl.HRDL_VOLTAGERANGE["HRDL_2500_MV"]
                except TypeError:
                    self.vrange[i] = 0
                    vrange = hrdl.HRDL_VOLTAGERANGE["HRDL_2500_MV"]

                self.status[f'activeChannel_{i}'] = hrdl.HRDLSetAnalogInChannel(self.chandle, i, 1, vrange, 1)
                assert_pico2000_ok(self.status[f'activeChannel_{i}'])

                # Get min and max adc values
                # Setup pointers
                self.minAdc[i] = ctypes.c_int32(0)
                self.maxAdc[i] = ctypes.c_int32(0)

                # Get min/max outputs for each channel
                self.status[f'minMaxAdcCounts_{i}'] = hrdl.HRDLGetMinMaxAdcCounts(self.chandle, ctypes.byref(self.minAdc[i]), ctypes.byref(self.maxAdc[i]), i)
                assert_pico2000_ok(self.status[f'minMaxAdcCounts_{i}'])

    def _updateMeta(self,channel):
        '''
        Internal function used to update class meta data and set new classes.

        Used during adding of new class.
        '''
        if self.active == True:
            # Shut everything down to reassign channels without errors
            self.shutdown()
        
        # Order channels
        channel = {key: val for key, val in sorted(channel.items(), key = lambda ele: ele[0])}
        self.channel = list(channel.keys()) # Set input channel here
        self.coefficients = channel
        self.numchannels = len(self.channel)

        # Restart everything - channels setup in _startup
        self._startup()

    def _setBuffer(self,newSize=1) -> None:
        '''
        Check buffer and update
        -----------------------

        Set new buffer size if requested buffer is different to the current buffer
        '''
        if newSize == self.buffer_size:
            pass
        else:
            self.buffer_size = newSize
        
        self.overflow = ctypes.c_int16(0)
        self.values = (ctypes.c_int32 * (self.buffer_size * self.numchannels))()
        self.times = (ctypes.c_int32 * (self.buffer_size * self.numchannels))()

    def _getBlock(self,bufferRequest):
        '''
        Function using ADC block/window method (0 or 1) to extract data
        ---------------------------------------------------

        Number of samples to take in the block/window\n
        buffer_size -> 4\n\n
        '''
        # Check and set buffer size
        self._setBuffer(bufferRequest)

        # Start sampling with BM_BLOCK (0) method
        self.status["collectingSamples"] = hrdl.HRDLRun(self.chandle, self.buffer_size, 0)
        assert_pico2000_ok(self.status["collectingSamples"])

        # While loop to pause program while data is collected
        while hrdl.HRDLReady(self.chandle) == 0: 
            sleep(0.05)

        '''
        ## Get values
        Setup pointer to output location with correct size
        Give pointers to ADC method to save output data to
        '''
        self.status["getTimesAndValues"] = hrdl.HRDLGetTimesAndValues(self.chandle, ctypes.byref(self.times), ctypes.byref(self.values), ctypes.byref(self.overflow), self.buffer_size)
        assert_pico2000_ok(self.status["getTimesAndValues"])

        return self.overflow, self.values, self.times

    def _getWindow(self,bufferRequest):
        '''
        Function using ADC window method (1) to extract data
        ----------------------------------------------------

        Number of samples to take in the block
        buffer_size -> 4
        '''
        return self._getStream(bufferRequest,method=1)

    def _getStream(self,bufferRequest,method=2):
        '''
        Function using ADC stream method (2) to extract data
        ----------------------------------------------------

        Buffer size
        buffer_size -> 5
        '''
        # Check and set buffer size
        self._setBuffer(bufferRequest)
        
        # Start sampling with BM_STREAM (2) method if not already started
        try:
            if self.streaming == 1:
                pass
            else:
                self._startStream(method)
        except TypeError:
            self._startStream(method)

        # While loop to pause program while data is collected
        while hrdl.HRDLReady(self.chandle) == 0:
            sleep(0.05)

        '''
        ## Get values
        Setup pointer to output location with correct size
        Give pointers to ADC method to save output data to
        '''
        # Get values and times
        self.status["getTimesAndValues"] = hrdl.HRDLGetTimesAndValues(self.chandle, ctypes.byref(self.times), ctypes.byref(self.values), ctypes.byref(self.overflow), self.buffer_size)
        assert_pico2000_ok(self.status["getTimesAndValues"])

        return self.overflow, self.values, self.times

    def _startWindow(self):
        '''
        Function using ADC window method (1) to extract data
        ----------------------------------------------------

        Use adc24._startStream() to begin stream.
        Once the stream has been started use adc24._getStream() to collect samples from buffer.
        '''
        # Start sampling with BM_WINDOW (1) method
        self._startStream(method=1)

    def _stopWindow(self):
        '''
        Stop streaming data from ADC20/24 device
        ----------------------------------------

        Use adc24._stopStream() to begin stream.
        Once the stream has been started use adc24._getStream() to collect samples from buffer.
        '''
        # Start sampling with BM_WINDOW (1) method
        self._stopStream()

    def _startStream(self,method=2):
        '''
        Function using ADC stream method (2) to extract data
        ----------------------------------------------------

        Use adc24._startStream() to begin stream.
        Once the stream has been started use adc24._getStream() to collect samples from buffer.
        '''
        # Start sampling with BM_WINDOW (1) method
        self.status["collectingSamples"] = hrdl.HRDLRun(self.chandle, self.buffer_size, method)
        assert_pico2000_ok(self.status["collectingSamples"])

        sleep(self.buffer_size*(self.sampleInterval_ms/1000))

        self.streaming = 1
    
    def _stopStream(self):
        '''
        Stop streaming data from ADC20/24 device
        ----------------------------------------

        Use adc24._stopStream() to begin stream.
        Once the stream has been started use adc24._getStream() to collect samples from buffer.
        '''
        # Start sampling with BM_WINDOW (1) method
        hrdl.HRDLStop(self.chandle)
        self.streaming = 0

    def _resetStream(self):
        '''
        Stop and start the stream to reset clock on ADC.
        Restart of stream is implied as it is called when collect is first called.
        '''
        self._stopStream()
        self._setBuffer()

    def _dataframe(self,data,times):
        '''
        Convert set of times and data into a pandas dataframe
        -----------------------------------------------------
        '''
        df_dict = {}
        df_dict['Time'] = times

        for key in data:
            df_dict[key] = data[key]
            
        df = pd.DataFrame(df_dict)

        return df

    def addCh(self,chDict={},vrange={}):
        '''
        Add channels to device
        ----------------------

        Coefficients specified by the dictionary.
        Repeating an already defined channel will update the coefficients.
        '''
        if not chDict:
            print(f'ERROR: No channels passed to addCh(). No change made.')
            pass
        elif type(chDict) != dict:
            if type(chDict) == list:
                print(f'WARNING: List passed. Channels {chDict} will output volts.')
                for i in chDict:
                    # If vrange has been passed, add details to vvrange variable
                    if not vrange:
                        if i in vrange:
                            self.vrange[i] = vrange[i]
                            self.coefficients[i] = [0,self.vrOptions[self.vrange[i]]]
                        else:
                            self.coefficients[i] = [0,2.5]
                    else:
                        self.coefficients[i] = [0,2.5]
                
                self._updateMeta(self.coefficients)
            else:
                print(f'ERROR: Must pass a dictionary or list to add channels. Type "{type(chDict)}" has instead been passed.')
                pass
        else:
            for i in chDict:
                # Check type and length of the input channel dictionary
                try:
                    assert type(chDict[i]) == list
                    if len(chDict[i]) == 2:
                        pass
                    else:
                        raise ValueError

                except AssertionError:
                    print(f'ERROR: New channel {i} incompatible. Must pass a LIST with 2 elements, method instead received {type(chDict[i])}.\nCHANNEL {i} NOT ADDED.')
                    continue
                except ValueError:
                    print(f'WARNING: No coefficients passed on channel {i} -> added as voltmeter.')
                    if not vrange:
                        if i in vrange:
                            self.vrange[i] = vrange[i]
                            self.coefficients[i] = [0,self.vrOptions[self.vrange[i]]]
                            continue
                        else:
                            self.coefficients[i] = [0,2.5]
                            continue
                    else:
                        self.coefficients[i] = [0,2.5]
                        continue
                
                # Add coefficients to channel
                self.coefficients[i] = chDict[i]

                # If vrange has been passed, add details to vvrange variable
                if not vrange:
                    if i in vrange:
                        self.vrange[i] = vrange[i]

            self._updateMeta(self.coefficients)

    def rmCh(self,chList=[],quiet=0):
        '''
        Remove channels from device
        ---------------------------

        Channel numbers to remove are passed as a list to this method.
        Channel numbers in the list that are not assigned to the device will be ignored.
        '''
        if not chList:
            if quiet == 0:
                print(f'ERROR: No channels passed to rmCh(). No change made.')
            else:
                pass
        else:
            for i in chList:
                try:
                    assert i in self.coefficients
                except AssertionError:
                    print(f'ERROR: Channel {i} not originally assigned. Nothing to remove.')
                self.coefficients.pop(i)
            self._updateMeta(self.coefficients)

    def modCh(self,chDict={},vrange={}):
        '''
        Replace all existing channel definitions
        ----------------------------------------

        All existing channels are replaced with those specified in the dictionary passed to this method.
        '''
        if not chDict:
            print(f'ERROR: No channels passed to modCh(). No change made.')
            pass
        elif type(chDict) != dict:
            if type(chDict) == list:
                self.rmCh(self.channel,quiet=1)
                self.addCh(chDict=chDict,vrange=vrange)
            else:
                print(f'ERROR: Must pass a dictionary or list to add channels. Type "{type(chDict)}" has instead been passed.')
                pass
        else:
            self.rmCh(self.channel,quiet=1)
            self.addCh(chDict=chDict,vrange=vrange)

    def all_out(self,buffer_size=2):
        '''
        (NEBULA LEGACY) Collect a data point for all inputs
        ---------------------------------------------------
        '''

        # Return arrays of time, pressure and flow rate
        overflow, values, times = self._getBlock(bufferRequest=buffer_size)
        
        '''
        ## Convert the output data from a ctype array to a real value. 
        ADC outputs data from input channels alternately. Slicing [start:end:interval] used to pick every other data point.
        Conversion equation turns raw ADC output into a physical value
        '''
        values_out = {}
        times_out = {}

        for n in range(self.numchannels):
            ch = self.channel[n]
            x0 = self.coefficients[ch][0]
            x1 = self.coefficients[ch][1]

            values_out[self.channel[n]] = np.around(((np.average(np.ctypeslib.as_array(values[n::2])) * x1/self.maxAdc[ch].value) + x0), decimals=4)
            times_out[self.channel[n]] = np.around(np.average(np.ctypeslib.as_array(times[n::2])/1000),decimals=4)

        return values_out, times_out
    
    def _reduceBuffer(self,values,times):
        '''
        Take buffer from stream and strip empty values
        '''
        values_out = {ch:[val for idx, val in enumerate(values[ch]) if ((idx == 0) or (round(times[idx],3) != 0.0))] for ch in values.keys()}
        times = [val for i, val in enumerate(times) if ((i == 0) or (round(val,3) != 0.0))]
        
        return values_out,times

    def bufferCollect(self,delayT=1,_delayTolerance=0.5,_print=False):
        '''
        Function to collect and manage buffer during a pause.
        Returns dataframe of output.
        '''
        stopT = delayT-_delayTolerance

        t0 = time()
        now = 0.0

        dataOut = self.collect(nsamples=10,method='stream',dataframe=True)

        while now < stopT:
            try:
                sleep(_delayTolerance)

                dataOut = pd.concat([dataOut,self.collect(nsamples=10,method='stream',dataframe=True)],ignore_index=True)
                
                if _print==True:
                    print(f'({now:.1f}/{delayT}) Pressure = {float(dataOut.iloc[-1,2]):.2f} bar    Flow rate = {float(dataOut.iloc[-1,15]):.2f} ul/min        ',end='\r')
                
                now = time() - t0

            except KeyboardInterrupt:
                break
        
        return dataOut

    def collect(self,nsamples=1,method='block',reset=0,dataframe=False):
        '''
        Collect data from ADC using specified method
        --------------------------------------------

        METHOD:\n
        'block'     ->  Collect block of data specified with nsamples per channel.\n
        'window'    ->  Collect windowed block of data with nsamples per channel.\n
        'stream'    ->  Collect continuous stream of data. First call begins stream.\n
         \n
        [NOT YET SUPPORTED] \n
        'single'    ->  Get a single value for each channel.\n \n 

        
        Optionally output data from block as a dataframe with the format:
        Time | Ch1 | Ch2 | ...
        ----------------------
             |     |     |    
             |     |     |
        '''
        try:
            if self.active == False:
                raise ValueError
            assert self.numchannels != 0
        except AssertionError:
            print(f'WARNING: No channels assigned. Used addCh() to add channels to this device.')
            return
        except ValueError:
            print(f'ERROR: Device inactive. Use wake() to activate device.')
            return

        try:
            if reset == 0:
                pass
            else:
                self._resetStream()
        except:
            self._resetStream()
        
        methods={
            'block':self._getBlock,
            'window':self._getWindow,
            'stream':self._getStream}
        
        overflow, values, times = methods[method](nsamples)
        
        '''
        ## Convert the output data from a ctype array to a real value. 
        ADC outputs data from input channels alternately. Slicing [start:end:interval] used to pick every other data point.
        Conversion equation turns raw ADC output into a physical value
        '''
        values_out = {}

        times_out = []
        times_out = np.ctypeslib.as_array(times[:nsamples])/1000

        for n in range(self.numchannels):
            ch = self.channel[n]
            x0 = self.coefficients[ch][0]
            x1 = self.coefficients[ch][1]

            values_out[self.channel[n]] = ((np.ctypeslib.as_array(values[n::self.numchannels]) * x1/self.maxAdc[ch].value) + x0)    

        if method == 'stream':
            values_out, times_out = self._reduceBuffer(values_out,times_out)

        if dataframe == True:
            df = self._dataframe(values_out, times_out)
            return df
        else:    
            return values_out, times_out

    def stop(self):
        '''
        Stop streams
        '''
        self._resetStream()
        
    def printCoeffs(self):
        '''
        Print out the coefficients used to convert the raw ADC value to an output in analytical form (y = x1 * x + x0).
        '''
        try:
            assert self.numchannels != 0
        except AssertionError:
            print(f'No channels assigned. Used addCh() to add channels to this device.')
            pass

        print(f'Coefficients and voltage ranges used for channels {self.channel}')
        for i in self.channel:
            
            if self.coefficients[i][0] >= 0:
                sign = '+ '
            else:
                sign = '- '

            print(f'Channel {i}  \t|  y = {self.coefficients[i][1]} * x {sign}{abs(self.coefficients[i][0])} \t|  Voltage range = {[key for key, val in hrdl.HRDL_VOLTAGERANGE.items() if val == self.vrange[i]]}')

    def reset(self):
        '''
        Keep unit open but reset all channels
        '''
        self._updateMeta({})
    
    def wake(self):
        '''
        Open a dormant device
        '''
        if self.active == False:
            self._startup()
        else:
            print(f'WARNING: Device already awake.')

    def ready(self):
        '''
        Return 1 if ready to collect data, 0 if not ready
        '''
        return hrdl.HRDLCloseUnit(self.chandle)

    def shutdown(self):
        '''
        Safely shutdown the ADC using the HRDLCloseUnit command 
        '''
        try:
            # Stop collecting, if collecting
            self._stopStream()
        except:
            pass
        
        if self.active == True:
            # Close unit
            self.status["closeUnit"] = hrdl.HRDLCloseUnit(self.chandle)
            assert_pico2000_ok(self.status["closeUnit"])
            self.active = False
        else:
            print(f'WARNING: Device already shutdown.')

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
        fit_data_path = f'{rpiDevices.__path__[0]}/Devices/fit_data.npy'
        self.fitdata = np.load(fit_data_path,allow_pickle=True)
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
        sleep(0.5)
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

class pce:
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

class imu:
    '''
    Class for the handling of the BNO055 9-DOF IMU with RPi
    -------------------------------------------------------
    \n
    "Temperature: {} degrees C".format(sensor.temperature)\n
    "Temperature: {} degrees C".format(temperature())\n

    "Accelerometer (m/s^2): {}".format(sensor.acceleration)\n

    "Magnetometer (microteslas): {}".format(sensor.magnetic)\n

    "Gyroscope (rad/sec): {}".format(sensor.gyro)\n

    "Quaternion: {}".format(sensor.quaternion)\n

    "Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration)\n

    "Gravity (m/s^2): {}".format(sensor.gravity)\n\n

    UPDATES REQUIRED:
    - Support for IMU functions as outputs
    - KARMAN filter for inertial navigation?
    '''

    def __init__(self):
        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.last_val = 0xFFFF

        self.functions = {
            'euler':lambda self: self.sensor.euler,
            'acceleration':lambda self: self.sensor.acceleration,
            'gyro':lambda self: self.sensor.gyro,
            'magnetic':lambda self: self.sensor.magnetic,
            'gravity':lambda self: self.sensor.gravity,
            'quaternion':lambda self: self.sensor.quaternion,
            'linear acceleration':lambda self: self.sensor.linear_acceleration,
            'temperature':lambda self: self.sensor.temperature,
        }

        self.euler = lambda: self.sensor.euler
        self.acceleration = lambda: self.sensor.acceleration
        self.gyro = lambda: self.sensor.gyro
        self.magnetic = lambda: self.sensor.magnetic
        self.gravity = lambda: self.sensor.gravity
        self.quarternion = lambda: self.sensor.quaternion
        self.linear_acceleration = lambda: self.sensor.linear_acceleration
        self.temperature = lambda: self.sensor.temperature

    def print(self,val='euler'):

        func = self.functions[val]
        
        try:
            while True:
                sign = {}
                out = func(self)
                try:
                    for i in range(3):
                        if out[i] > 0:
                            sign[i] = '+'
                        else:
                            sign[i] = '-'
                    print(f"{val}:  ({sign[0]}{abs(out[0]):06.2f}, {sign[1]}{abs(out[1]):06.2f}, {sign[2]}{abs(out[2]):06.2f})                         ",end='\r')
                except TypeError:
                    pass
        except KeyboardInterrupt:
            print(f'\n------------------------------------------- \n Closing program (no save) \n')

def README():
    '''
    Print details of the rpiDevices package.
    '''
    print(__doc__)

def help(function):
    print(function.__doc__)

def new():
    os.system('clear && toilet -t rpiDevices')
    README()