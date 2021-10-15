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
from bashplotlib.histogram import plot_hist

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
        try:
            data.append(ADC.collect(1))
            timeData.append(time.time() - t0)
            print(f'({time.time() - t0:.1f}/{testT}) Pressure = {float(data[-1][0][2]):.2f} ({pressure:.2f}) bar    Flow rate = {float(data[-1][0][15]):.2f} ul/min        ',end='\r')
            time.sleep(sampleT)
        except KeyboardInterrupt:
            break
        
    PR.set_P(-1)

    data = [i[0] for i in data] # Remove time placeholder
    data = [ap(dic,'Time',timeData[i]) for i, dic in list(enumerate(data))] # Use measured time as Time data column
    dataFrame = pd.concat([pd.DataFrame(i) for i in data],ignore_index=True) # Combine measurements into one dataframe
    dataFrame.columns = ['Pressure (bar)','Flowrate (ul/min)','Time']

    print(f'\nStep pressure change results')
    print(f'----------------------------')
    print(dataFrame.describe())

    dataFrame.loc[:,['Time',2]].to_csv('./StepDataPressure.csv',index=False,header=False)
    dataFrame.loc[:,['Time',15]].to_csv('./StepDataFlow.csv',index=False,header=False)

    with open('./StepDataPressure.csv') as f:
        plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Step response - Pressure')
    with open('./StepDataFlow.csv') as f:
        plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Step response - Flow rate')

    return dataFrame

def hold(PR,ADC,pressure=2.0,testT=5.0,sampleT=0.5):
    '''
    Record change in pressure and flow rate over a period of time
    -------------------------------------------------------------

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

    PR.set_P(pressure)

    time.sleep(5)
    data = [ADC.collect(1)]
    t0 = time.time()
    timeData = [time.time() - t0]

    while time.time() - t0 < testT:
        try:
            data.append(ADC.collect(1))
            timeData.append(time.time() - t0)
            print(f'({time.time() - t0:.1f}/{testT}) Pressure = {float(data[-1][0][2]):.2f} ({pressure:.2f}) bar    Flow rate = {float(data[-1][0][15]):.2f} ul/min        ',end='\r')
            time.sleep(sampleT)
        except KeyboardInterrupt:
            break
        
    PR.set_P(-1)

    data = [i[0] for i in data] # Remove time placeholder
    data = [ap(dic,'Time',timeData[i]) for i, dic in list(enumerate(data))] # Use measured time as Time data column
    dataFrame = pd.concat([pd.DataFrame(i) for i in data],ignore_index=True) # Combine measurements into one dataframe
    dataFrame.columns = ['Pressure (bar)','Flowrate (ul/min)','Time']

    print(f'\nStep pressure change results')
    print(f'----------------------------')
    print(dataFrame.describe())

    dataFrame.loc[:,['Time',2]].to_csv('./HoldDataPressure.csv',index=False,header=False)
    dataFrame.loc[:,['Time',15]].to_csv('./HoldDataFlow.csv',index=False,header=False)

    with open('./HoldDataPressure.csv') as f:
        plot_histogram(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Hold response - Pressure')
    with open('./HoldDataFlow.csv') as f:
        plot_histogram(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Hold response - Flow rate')

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
    dataFrame.columns = ['Pressure (bar)','Flowrate (ul/min)','Time']

    print(f'\nRamp pressure change results')
    print(f'----------------------------')
    print(dataFrame.describe())

    dataFrame.loc[:,['Time',2]].to_csv('./RampDataPressure.csv',index=False,header=False)
    dataFrame.loc[:,['Time',15]].to_csv('./RampDataFlow.csv',index=False,header=False)

    with open('./RampDataPressure.csv') as f:
        plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Ramp response - Pressure')
    with open('./RampDataFlow.csv') as f:
        plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Ramp response - Flow rate')

    return dataFrame

def impulse(PR,ADC,pressure=5.0,testT=5.0):
    '''
    Record response to pressure impulse
    -----------------------------------

    DEFAULTS
    --------
    pressure    -> 5 bar
    testT       -> 5 s
    '''
    def ap(dicti,key,value):
        '''
        Function to return dictionary with new key value pair added
        '''
        dicti[key] = value
        return dicti

    data = [ADC.collect(1)]

    t0 = time.time()
    now = time.time() - t0

    timeData = [now]

    PR.set_P(pressure)
    time.sleep(0.1)
    PR.set_P(-1)

    now = time.time() - t0

    while now < testT:
        try:
            data.append(ADC.collect(1))
            timeData.append(now)
            # print(f'({time.time() - t0:.1f}/{testT}) Pressure = {float(data[-1][0][2]):.2f} ({pressure:.2f}) bar    Flow rate = {float(data[-1][0][15]):.2f} ul/min        ',end='\r')
            now = time.time() - t0
        except KeyboardInterrupt:
            break
        
    PR.set_P(-1)

    data = [i[0] for i in data] # Remove time placeholder
    data = [ap(dic,'Time',timeData[i]) for i, dic in list(enumerate(data))] # Use measured time as Time data column
    dataFrame = pd.concat([pd.DataFrame(i) for i in data],ignore_index=True) # Combine measurements into one dataframe
    dataFrame.columns = ['Pressure (bar)','Flowrate (ul/min)','Time']

    print(f'\nStep pressure change results')
    print(f'----------------------------')
    print(dataFrame.describe())

    dataFrame.loc[:,['Time',2]].to_csv('./StepDataPressure.csv',index=False,header=False)
    dataFrame.loc[:,['Time',15]].to_csv('./StepDataFlow.csv',index=False,header=False)

    with open('./StepDataPressure.csv') as f:
        plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Step response - Pressure')
        plot_hist(f=f,height=20,pch='x',colour='white',title='Step response - Pressure',xlab=True,nosummary=False)
    with open('./StepDataFlow.csv') as f:
        plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Step response - Flow rate')
        plot_hist(f=f,height=20,pch='x',colour='white',title='Step response - Pressure',xlab=True,nosummary=False)

    return dataFrame

def main(testT=10,pressure=3,sampleT=0):
    print(f'Running step and ramp test at {pressure} bar for {testT} seconds with a sample period of {sampleT} seconds.')
    print(f'Initialising devices ...')
    PR = vppr()
    ADC = adc24(channel='nebula')

    print(f'Done initialising!\nRunning tests.')

    output_df = {}

    # HOLD RESPONSE
    output_df['Hold'] = hold(PR,ADC,pressure=pressure,testT=testT,sampleT=sampleT)

    # STEP RESPONSE
    # output_df['Step'] = step(PR,ADC,pressure=pressure,testT=testT,sampleT=sampleT)

    # IMPULSE RESPONSE
    output_df['Impulse'] = impulse(PR,ADC,testT=testT)

    # RAMP RESPONSE
    # output_df['Ramp'] = ramp(PR,ADC,pressure=pressure,rampT=5,testT=testT,sampleT=sampleT)
    
    ADC.shutdown()

    return output_df

if __name__ == '__main__':
    main()