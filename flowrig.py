# -*- coding: utf-8 -*-
"""
FLOWRIG
-------

Created 2021-04-23
@author: Jordan Eriksen
"""

from Devices.rpi import *


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
                    data_out.iloc[-n:].to_csv(f'{fileloc}/{filename}_t{i+1}.csv',index=False) 
                    data_out.iloc[-n:].to_pickle(f'{fileloc}/{filename}_t{i+1}.pkl')

                    # Save data from all repeats in one file.
                    data_out.to_csv(f'{fileloc}/{filename}.csv',index=False)
                    data_out.to_pickle(f'{fileloc}/{filename}.pkl')
                
                else: # If there are still some repeats to conduct just save the repeat, not the master .csv and.pkl
                    data_out.iloc[-n:].to_csv(f'{fileloc}/{filename}_t{i+1}.csv',index=False)
                    data_out.iloc[-n:].to_pickle(f'{fileloc}/{filename}_t{i+1}.pkl')

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
    subprocess.run(f'mkdir ../Data_{socket.gethostname()}/{datetime.datetime.now().date()}_{identifier}', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    fileloc=f'../Data_{socket.gethostname()}/{datetime.datetime.now().date()}_{identifier}'

    # Run tests - specify number of repeats as argument in terminal
    test_run(picolog, regulator, pressure, data_out, identifier,fileloc)

    # Save output to git repository
    os.system(f'cd ../Data_{socket.gethostname()} && git add -A && git commit -m "{identifier}" && cd ..')