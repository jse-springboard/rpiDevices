'''
Author: Jordan Eriksen
Date: 2021-10-14

Demand a change in pressure and measure response
------------------------------------------------

INPUTS
------
sampleT     -> [float] Sample period (seconds)
testT       -> [float] Total time to run test for (seconds)
pressure    -> [float] Demand pressure change (bar)

OUTPUTS
-------
flowRate    -> [array] Flow rate measured (ul/min)
pressure    -> [array] Actual pressure recorded (bar)
summary     -> [DataFrame] Statistical summary of the results
'''

from rpiDevices.rpi import adc24, vppr
import time
import pandas as pd
from bashplotlib.scatterplot import plot_scatter

def step(PR,ADC,pressure=2.0,testT=5.0,sampleT=0.5):
    '''
    Record response to a step change in pressure
    --------------------------------------------

    DEFAULTS
    --------
    pressure    -> 2 bar
    testT       -> 5 s
    sampleT     -> 0.2 s
    '''
    def ap(dicti,key,value):
        '''
        Function to return dictionary with new key value pair added
        '''
        dicti[key] = value
        return dicti

    data = [ADC.collect(1)]

    t0 = time.time()
    timeData = [time.time() - t0]

    PR.set_P(pressure)

    while time.time() - t0 < testT:
        data.append(ADC.collect(1))
        timeData.append(time.time() - t0)
        time.sleep(sampleT)
    
    PR.set_P(-1)

    data = [i[0] for i in data] # Remove time placeholder
    data = [ap(dic,'Time',timeData[i]) for i, dic in list(enumerate(data))] # Use measured time as Time data column
    dataFrame = pd.concat([pd.DataFrame(i) for i in data],ignore_index=True) # Combine measurements into one dataframe

    return dataFrame

def ramp(PR,ADC,pressure=2.0,rampT=5.0,testT=5.0,sampleT=0.2):
    '''
    Record response to a ramped change in pressure
    ----------------------------------------------

    DEFAULTS
    --------
    pressure    -> 2 bar
    rampT       -> 5 s
    testT       -> 10 s
    sampleT     -> 0.2 s
    '''
    holdT = testT - rampT
    pressureGrad = pressure/rampT

    def ap(dicti,key,value):
        '''
        Function to return dictionary with new key value pair added
        '''
        dicti[key] = value
        return dicti

    data = [ADC.collect(1)]

    t0 = time.time()

    timeData = [time.time() - t0]

    while time.time() - t0 < rampT:
        PR.set_P(pressureGrad*(time.time() - t0))
        data.append(ADC.collect(1))
        timeData.append(time.time() - t0)
        time.sleep(sampleT)

    t1 = time.time()

    while time.time() - t1 < holdT:
        data.append(ADC.collect(1))
        timeData.append(time.time() - t0)
        time.sleep(sampleT)
    
    PR.set_P(-1)

    data = [i[0] for i in data] # Remove time placeholder
    data = [ap(dic,'Time',timeData[i]) for i, dic in list(enumerate(data))] # Use measured time as Time data column
    dataFrame = pd.concat([pd.DataFrame(i) for i in data],ignore_index=True) # Combine measurements into one dataframe

    return dataFrame

def main(testT=10,pressure=3,sampleT=0):
    print(f'Running step and ramp test at {pressure} bar for {testT} seconds with a sample period of {sampleT} seconds.')
    print(f'Initialising devices ...')
    PR = vppr()
    ADC = adc24(channel='nebula')

    print(f'Done initialising!\nRunning tests.')

    stepDataFrame = step(PR,ADC,pressure=pressure,testT=testT,sampleT=sampleT)

    print(f'\nStep pressure change results')
    print(f'----------------------------')
    print(stepDataFrame.describe())

    stepDataFrame.loc[:,['Time',2]].to_csv('./StepDataPressure.csv',index=False,header=False)
    stepDataFrame.loc[:,['Time',15]].to_csv('./StepDataFlow.csv',index=False,header=False)
    with open('./StepDataPressure.csv') as f:
        plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Step response - Pressure')
    with open('./StepDataFlow.csv') as f:
        plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Step response - Flow rate')


    # rampDataFrame = ramp(PR,ADC,pressure=pressure,rampT=5,testT=testT,sampleT=sampleT)

    # print(f'\nRamp pressure change results')
    # print(f'----------------------------')
    # print(rampDataFrame.describe())

    # rampDataFrame.loc[:,['Time',15]].to_csv('./RampData.csv',index=False,header=False)   
    # with open('./RampData.csv') as f:
    #     plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Ramp response')

    
    ADC.shutdown()

if __name__ == '__main__':
    main()