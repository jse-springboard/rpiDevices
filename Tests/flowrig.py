# -*- coding: utf-8 -*-
"""
FLOWRIG
-------

Created 2021-04-23
@author: Jordan Eriksen
"""

from rpirig import *

# import ctypes
# import numpy as np
# import pandas as pd
# import datetime
# from time import sleep
# import time
# import os
# import sys
# import subprocess
# import socket

# from picosdk.picohrdl import picohrdl as hrdl
# from picosdk.functions import assert_pico2000_ok

# import RPi.GPIO as gpio

# class adc24:
#     '''
#     Class for the handling of a Picolog ADC20/24 using PicoSDK
#     ----------------------------------------------------------

#     Requires a powered USB hub to power the ADC.
#     '''

#     def __init__(self, channel=[2,15]):
#         # Create chandle and status ready for use
#         self.chandle = ctypes.c_int16()
#         self.status = {}
#         self.numsamples = 1
#         self.channel = channel # Set input channel here

#         self._startup()

#     def _startup(self):
#         # Open unit
#         self.status["openUnit"] = hrdl.HRDLOpenUnit()
#         assert_pico2000_ok(self.status["openUnit"])
            
#         self.chandle=self.status["openUnit"]

#         # Set mains noise rejection
#         # Reject 50 Hz mains noise
#         self.status["mainsRejection"] = hrdl.HRDLSetMains(self.chandle, 0)
#         assert_pico2000_ok(self.status["mainsRejection"])

#         # Open ADC channel
#         vrange = hrdl.HRDL_VOLTAGERANGE["HRDL_2500_MV"]
#         self.status["activeChannel_PT"] = hrdl.HRDLSetAnalogInChannel(self.chandle, self.channel[0], 1, vrange, 1)
#         assert_pico2000_ok(self.status["activeChannel_PT"])
#         self.status["activeChannel_FM"] = hrdl.HRDLSetAnalogInChannel(self.chandle, self.channel[1], 1, vrange, 1)
#         assert_pico2000_ok(self.status["activeChannel_FM"])

#         #Get min and max adc values
#         # Setup pointers
#         self.minAdc_P = ctypes.c_int32(0) # Min for pressure transducer
#         self.minAdc_F = ctypes.c_int32(0) # Min for flow meter
#         self.maxAdc_P = ctypes.c_int32(0) # Max for pressure transducer
#         self.maxAdc_F = ctypes.c_int32(0) # Max for flow meter

#         # Get min/max outputs for pressure transducer
#         self.status["minMaxAdcCounts_PT"] = hrdl.HRDLGetMinMaxAdcCounts(self.chandle, ctypes.byref(self.minAdc_P), ctypes.byref(self.maxAdc_P), self.channel[0])
#         assert_pico2000_ok(self.status["minMaxAdcCounts_PT"])
#         # Get min/max outputs for flow meter
#         self.status["minMaxAdcCounts_FM"] = hrdl.HRDLGetMinMaxAdcCounts(self.chandle, ctypes.byref(self.minAdc_F), ctypes.byref(self.maxAdc_F), self.channel[1])
#         assert_pico2000_ok(self.status["minMaxAdcCounts_FM"])
        
#         # Set sampling time interval
#         conversionTime = hrdl.HRDL_CONVERSIONTIME["HRDL_60MS"]
#         self.status["samplingInterval"] = hrdl.HRDLSetInterval(self.chandle, 121, conversionTime)
#         assert_pico2000_ok(self.status["samplingInterval"])

#     def _get_data(self,nums_=4):
#         '''
#         Function using ADC method to extract data
#         '''
#         # Start sampling with BM_BLOCK (0) method
#         self.status["collectingSamples"] = hrdl.HRDLRun(self.chandle, nums_, 0)
#         assert_pico2000_ok(self.status["collectingSamples"])

#         # While loop to pause program while data is collected
#         while hrdl.HRDLReady(self.chandle) == 0: 
#             sleep(0.05)

#         '''
#         ## Get values
#         Setup pointer to output location with correct size
#         Give pointers to ADC method to save output data to
#         '''
#         overflow = ctypes.c_int16(0)
#         values = (ctypes.c_int32 * (nums_ * 2))()
#         times = (ctypes.c_int32 * (nums_ * 2))()
#         self.status["getTimesAndValues"] = hrdl.HRDLGetTimesAndValues(self.chandle, ctypes.byref(times), ctypes.byref(values), ctypes.byref(overflow), nums_)
#         assert_pico2000_ok(self.status["getTimesAndValues"])

#         return overflow, values, times

#     def all_out(self,nums=4,offset_p=0.,offset_f=-0.105):
#         '''
#         Collect a data point for all inputs
#         '''
#         # Return arrays of time, pressure and flow rate
#         overflow, values, times = self._get_data(nums_=nums)
        
#         '''
#         ## Convert the output data from a ctype array to a real value. 
#         ADC outputs data from input channels alternately. Slicing [start:end:interval] used to pick every other data point.
#         Conversion equation turns raw ADC output into a physical value
#         '''
#         pressure = np.around(((np.average(np.ctypeslib.as_array(values[0::2])) * 21/self.maxAdc_P.value) + offset_p), decimals=4)
#         flowrate = np.around(((np.average(np.ctypeslib.as_array(values[1::2])) * 2.5/self.maxAdc_F.value * 32) + offset_f), decimals=4)

#         # Uncomment the below to output times as well as data
#         # times_p = np.around(np.average(np.ctypeslib.as_array(times[0::2])/1000),decimals=4)
#         # times_f = np.around(np.average(np.ctypeslib.as_array(times[1::2])/1000),decimals=4)

#         return pressure, flowrate #, times_p, times_f

#     def shutdown(self):
#         '''
#         Safely shutdown the ADC using the HRDLCloseUnit command 
#         '''
#         # Close unit
#         self.status["closeUnit"] = hrdl.HRDLCloseUnit(self.chandle)
#         assert_pico2000_ok(self.status["closeUnit"])

# class vppr:
#     '''
#     Class for the handling of a voltage proportional pressure regulator using a RPi
#     -------------------------------------------------------------------------------

#     Requires a DAC to convert PWM signal to a voltage.
#     Connected DAC has required output voltage (10V) and can run off a 15V supply.

#     Check that required supporting files are in the correct place:
#         - Requires a fit_data.npy file to manage the offset between demand pressure
#           and actual output pressure
#         - .//Setup//fit_data.npy
#     '''
#     def __init__(self,minout=0.0,maxout=10, pwm_freq=1.2e3, pwm_pin=12, mode='hw'):
#         self.minout = minout # Minimum output pressure set by VPPR
#         self.maxout = maxout # Maximum output pressure set by VPPR
#         self.pwm_freq = pwm_freq # 
#         self.pwm_pin = pwm_pin
#         self.mode = mode
        
#         # Load a fit curve to manage the offset between demand and actual output pressure
#         self.fitdata = np.load('.//Setup//fit_data.npy',allow_pickle=True)
#         self.fit_pressure = np.polynomial.Polynomial.fit(self.fitdata[0,:],self.fitdata[1,:],9)

#         # Define mode of operation. Default should be hardware ('hw').
#         if self.mode == 'sw':
#             self._startup_SW()
#         else:
#             self._startup()

#     def _startup_SW(self):
#         '''
#         Function starts the software handling of the PWM signal using RPi.GPIO module
#         '''
#         # Set pin numbering format
#         gpio.setmode(gpio.BCM)

#         # Setup pwm pin as output
#         gpio.setup(self.pwm_pin,gpio.OUT)

#         # Begin PWM
#         self.pwm = gpio.PWM(self.pwm_pin,self.pwm_freq)
#         self.pwm.start(0)

#     def _startup(self):
#         '''
#         Begin pigpio deamon to handle hardware PWM on the RPi. Output is supressed to reduce clutter.
#         '''
#         # Will raise error if daemon already running, error is supressed using stderr=subprocess.DEVNULL
#         subprocess.run(f'sudo pigpiod', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) 
#         sleep(0.2)

#         # Start PWM and set to 0
#         subprocess.run(f'pigs hp {self.pwm_pin} {int(self.pwm_freq)} {0}', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

#     def set_P(self,pressure=-1):
#         '''
#         Set the pressure of the regulator by mapping pwm duty -> DAC output -> pressure;
#         Voltage = PWM% x 0.1;
#         Pressure = Voltage * (P_max - P_min)/10 + P_min;
        
#         Hence PWM = ((Pressure - P_min)/(P_max - P_min))*100
#         '''
        
#         # Output pressure must be greater than the minimum output of the VPPR
#         if pressure < self.minout:
#             # Use pigs to change hardware PWM signal to 0 if pressure is too low
#             os.system(f'pigs hp {self.pwm_pin} {int(self.pwm_freq)} {int(0)}')

#         else:
#             # To reduce strain on the system, each pressure is reached by ramping up to the required pressure over 10 steps.
#             # Infer last demand pressure from the current PWM output using pigs gdc <pwm_pin> (GET DUTY CYCLE)
#             proc = subprocess.run(f'pigs gdc {self.pwm_pin}', shell=True, encoding='utf-8', stdout=subprocess.PIPE)

#             # Account for non-linearity of VPPR by using a lookup to determine demand pressure required to set actual pressure
#             # See AIV-00019-revA for details on this
#             pressure = self.fit_pressure(pressure)

#             # Calculate last demand p using the PWM signal
#             # Equation is the inverse of that at the top of the function
#             pressure_last = (float(proc.stdout)/1000000)*(self.maxout - self.minout) + self.minout
            
#             #Setup pressure ramp
#             pressure_ramp = np.linspace(pressure_last,pressure,10)

#             # Cunduct pressure ramp using the pigs hp method to set PWM output
#             for press in pressure_ramp:
#                 pwm_set = ((press - self.minout)/(self.maxout- self.minout))*100
#                 if round(pwm_set) > 100:
#                     pwm_set = 100
#                 else:
#                     pass
#                 os.system(f'pigs hp {self.pwm_pin} {int(self.pwm_freq)} {int(pwm_set*10000)}')
        
#     def _set_P_SW(self,pressure=-1):
#         '''
#         NOT USED - CHECK FUNCTIONALITY BEFORE CALLING

#         Uses the software PWM in RPI.GPIO instead of hardware. Less accurate and slower.

#         Set the pressure of the regulator by mapping pwm duty -> DAC output -> pressure;
#         Voltage = PWM% x 0.1;
#         Pressure = Voltage * (P_max - P_min)/10 + P_min;
        
#         Hence PWM = ((Pressure - P_min)/(P_max - P_min))*100
#         '''

#         if pressure < self.minout:
#             self.pwm.ChangeDutyCycle(0)
#         else:
#             # pressure = self.fitdata(pressure)
#             pwm_set = ((pressure - self.minout)/(self.maxout- self.minout))*100
#             self.pwm.ChangeDutyCycle(pwm_set)

#     def shutdown(self):
#         '''
#         Safely shutdown the VPPR
#         '''
#         if self.mode == 'hw': # If using hardware PWM, set dutycycle to 0 to shutdown PWM
#             self.set_P() # Calling set_P without an argument sets the dutycycle to 0
#         else: # If using software PWM, stop PWM and cleanup GPIO pins
#             self.set_P()
#             self.pwm.stop()
#             gpio.cleanup()

def get_data(adc24_, vppr_, pressure=0, t0=0, settlingtime=3, progress=['?','?']):
    '''
    Function to handle the calling of vppr and collection from adc
    '''

    # Call function in VPPR class to set the pressure for current data point
    vppr_.set_P(pressure)

    # Wait for a period of time to allow the system to settle
    time.sleep(settlingtime)

    # Collect datapoint from the ADC
    pressure_out, flowrate = adc24_.all_out() # pressure, flowrate, times_p, times_f = picolog.all_out()

    # Record the time of the data point (relative to the start of the test repeat)
    t = time.time() - t0

    # Define, print and return the datapoint as a dictionary (easier to turn into a DataFrame later)
    line = {'Demand_P':pressure, 'Pressure':pressure_out, 'Flow rate':flowrate, 'Time':t}
    print(f'({progress[0]}/{progress[1]}) Pressure = {pressure_out:.2f} ({pressure:.2f}) bar    Flow rate = {flowrate:.3f} ul/min    Time = {t:.0f} s',end='\r')
    return line

def test_run(adc24_, vppr_, pressure, data_out, identifier='TEST', fileloc='.//', numreps=1):
    '''
    Manage calling of each test run
    '''
    try:
        # Print number of repeats to be taken, this will raise an exception if no argument present, or if the argument is invalid
        print(f'Number of repeats: {int(sys.argv[1])}')

        # Set filename for all repeats to show the start time of the first test to enble tests to be matched later
        test_date = datetime.datetime.now().date()
        test_time = datetime.datetime.now().strftime("%H-%M")

        # For loop for each test repeat
        for i in range(int(sys.argv[1])):

            # Define filename based on current date and time, pressure range, and test ID
            filename = f'{test_date}_{test_time}_P_{pressure[0]}-{pressure[-1]}_{identifier}'
            print(f'\n\033[1mBeginning test ID {identifier}_t{i+1} \033[0m\nP_min = {pressure[0]};    P_max = {pressure[-1]}\n-------------------------------------------')
            
            # Run test
            try:
                # Record start time of the test and count number of datapoints collected
                t0 = time.time()
                n = 0

                # Cycle through the required pressures
                for p in pressure:
                    progress = [n+1,len(pressure)]
                    line_out = get_data(adc24_,vppr_,p,t0,progress=progress) # Call function to set pressure and record data at that data point
                    data_out = data_out.append(line_out, ignore_index=True) # Add datapoint to DataFrame
                    n += 1 # Count datapoint

                # Use KeyboardInterrupt to signal end of test repeat. Also allows user to cancel a repeat at anytime and handle the correctly
                raise KeyboardInterrupt

            except KeyboardInterrupt: # Handle shutdown of ADC and the VPPR and save data
                if i == max(range(int(sys.argv[1]))): # Check if most recent repeat was the last one required, if so shutdown and save master .csv and .pkl
                    adc24_.shutdown() 
                    vppr_.shutdown()

                    # Save data from this repeat. Get required datapoints using number taken in last test (n)
                    data_out.iloc[-n:].to_csv(f'{fileloc}//{filename}_t{i+1}.csv',index=False) 
                    data_out.iloc[-n:].to_pickle(f'{fileloc}//{filename}_t{i+1}.pkl')

                    # Save data from all repeats in one file.
                    data_out.to_csv(f'{fileloc}//{filename}.csv',index=False)
                    data_out.to_pickle(f'{fileloc}//{filename}.pkl')
                
                else: # If there are still some repeats to conduct just save the repeat, not the master .csv and.pkl
                    data_out.iloc[-n:].to_csv(f'{fileloc}//{filename}_t{i+1}.csv',index=False)
                    data_out.iloc[-n:].to_pickle(f'{fileloc}//{filename}_t{i+1}.pkl')

            # After each repeat, print out dataframe for that repeat
            print(f'\n------------------------------------------- \n{data_out.iloc[-n:]}')
            print(f'------------------------------------------- \n')

    except (IndexError,TypeError): # No argument given in the command line so use default value, defined by numreps. Algorithm the same as above
        print(f'Number of repeats: {numreps}')
        # Set filename for all repeats to show the start time of the first test to enble tests to be matched later
        test_date = datetime.datetime.now().date()
        test_time = datetime.datetime.now().strftime("%H-%M")

        for i in range(numreps):
            # Markers
            filename = f'{test_date}_{test_time}_P_{pressure[0]}-{pressure[-1]}_{identifier}'
            
            print(f'\n\033[1mBeginning test ID {identifier}_t{i+1} \033[0m\nP_min = {pressure[0]};    P_max = {pressure[-1]}\n-------------------------------------------')
            # Get data
            try:
                t0 = time.time()
                n = 0
                for p in pressure:
                    progress = [n+1,len(pressure)]
                    line_out = get_data(adc24_,vppr_,p,t0,progress=progress)
                    data_out = data_out.append(line_out, ignore_index=True)
                    n += 1
                raise KeyboardInterrupt
            except KeyboardInterrupt:
                if i == max(range(numreps)):
                    adc24_.shutdown()
                    vppr_.shutdown()
                    # print(n)
                    data_out.iloc[-n:].to_csv(f'{fileloc}//{filename}_t{i+1}.csv',index=False)
                    data_out.iloc[-n:].to_pickle(f'{fileloc}//{filename}_t{i+1}.pkl')

                    data_out.to_csv(f'{fileloc}//{filename}.csv',index=False)
                    data_out.to_pickle(f'{fileloc}//{filename}.pkl')
                else:
                    data_out.iloc[-n:].to_csv(f'{fileloc}//{filename}_t{i+1}.csv',index=False)
                    data_out.iloc[-n:].to_pickle(f'{fileloc}//{filename}_t{i+1}.pkl')


            print(f'\n------------------------------------------- \n{data_out.iloc[-n:]}')
            print(f'------------------------------------------- \n')

def _get_inputs(default = {'low':0.0, 'high':6.0, 'n':20}):
    '''
    Return minimum, maximum and number of pressure data points.
    If input is invalid or no input is given, default values will be taken by handling any ValueError exceptions.
    '''

    # Get lowest test pressure
    try:
        lowP_ = float(input(f'Minimum pressure (default = 0 bar): '))
        default['low'] = lowP_
    except ValueError:
        pass

    # Get highest test pressure 
    try:
        highP_ = float(input(f'Maximum pressure (default = 4 bar): '))
        default['high'] = highP_
    except ValueError:
        pass
    
    # Get number of tests
    try:
        n_ = int(input(f'Number of test points (default = 20): '))
        default['n'] = n_
    except ValueError:
        pass

    return default

if __name__ == '__main__':

    # Initialise instances of adc and vppr classes
    print('')
    print('\033[1mInitialising ADC and regulator\033[0m')
    picolog = adc24()
    regulator = vppr()

    # Setup empty data frame for results
    data_out = pd.DataFrame(columns=['Demand_P','Pressure','Flow rate','Time'])
    
    '''
    ## Define test ID
    Test ID either taken directly from the arguments passed from the command line, or if not present, ask operator for one
    '''
    try:
        identifier = str(sys.argv[2])
    except IndexError:
        identifier = str(input('Enter test identifier: '))
    print(f'------------------------------------------- \n')

    '''
    ## Define test points
    Activate default either from command line (arg[3] == 1) or ask operator for input
    '''
    try:
        # Get arguments from the command line. Raises IndexError if there is none, handle with except.
        if int(sys.argv[3]) == 1: # If arg is 1, select default test program
            pressure = np.linspace(0.0,6.0,20)
        else: 
            raise IndexError
    except (IndexError, TypeError): # Raised if either no arg[3] or type not int
        try:
            # Get operator input. Raises ValueError if default is selected. Handled by except.
            set_P = int(input(f'Set pressures? (ENTER FOR DEFAULT, "1" FOR CUSTOM): '))

            # If default not selected, get inputs from operator using _get_inputs function
            pressure_ = _get_inputs()

            # Organise required pressures into array
            pressure = np.linspace(pressure_['low'],pressure_['high'],pressure_['n'])

            '''

            INSERT HERE A WAY OF CONTROLLING THE PRESSURE RAMPING TO INCLUDE RAMP UP AND PARTIAL RAMP DOWN

            '''

        except ValueError: # Defaults selected
            pressure = np.linspace(0.0,6.0,20)

    # Make folder for test results
    subprocess.run(f'mkdir ./Data_{socket.gethostname()}/{datetime.datetime.now().date()}_{identifier}', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    fileloc=f'.//Data_{socket.gethostname()}//{datetime.datetime.now().date()}_{identifier}'

    # Run tests - specify number of repeats as argument in terminal
    test_run(picolog, regulator, pressure, data_out, identifier,fileloc)

    # Save output to git repository
    os.system(f'cd Data_{socket.gethostname()} && git add -A && git commit -m "{identifier}" && cd ..')