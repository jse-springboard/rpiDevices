# -*- coding: utf-8 -*-
"""
LEAKRIG
-------

Created 2021-05-27
@author: Jordan Eriksen
"""
from Devices.rpi import *


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
        subprocess.run(f'mkdir ../Data_{socket.gethostname()}/{test_date}_LEAKAGE_{identifier}', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        fileloc=f'../Data_{socket.gethostname()}/{test_date}_LEAKAGE_{identifier}'

        data.to_csv(f'{fileloc}/{filename}.csv',index=False)
        data.to_pickle(f'{fileloc}/{filename}.pkl')

        # Save output to git repository
        os.system(f'cd ../Data_{socket.gethostname()} && git add -A && git commit -m "{identifier}" && cd ..')
