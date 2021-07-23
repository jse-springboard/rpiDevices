# -*- coding: utf-8 -*-
"""
SPRAY TIME
----------

Created 2021-06-01
@author: Jordan Eriksen
"""
from Devices.rpi import tof


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
            try:
                for i in range(15):
                    if i == 0:
                        _time = [time.time() - t0]
                        _dist = [_tof.range()]
                    else:
                        _time.append(time.time() - t0)
                        _dist.append(_tof.range())

                _grad = np.polyfit(_time,_dist,1)
                line = pd.DataFrame({'Time':np.mean(_time),'Distance':np.mean(_dist),'delta':_grad[0]}, 
                                    columns=['Time','Distance','delta'], 
                                    index=[0])

                df = df.append(line, ignore_index=True)
            except KeyboardInterrupt:
                break
                
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