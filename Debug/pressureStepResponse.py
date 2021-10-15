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

def delayCollect(ADC,delayT=5,_delayTolerance=0.05,_print=False):
    '''
    Function to collect and manage buffer during a pause.
    Returns dataframe of output.
    '''
    stopT = delayT-_delayTolerance

    t0 = time.time()
    now = 0.0

    dataOut = ADC.collect(nsamples=10,method='stream',dataframe=True)

    while now < stopT:
        try:
            time.sleep(_delayTolerance)

            dataOut = pd.concat([dataOut,ADC.collect(nsamples=10,method='stream',dataframe=True)],ignore_index=True)
            
            if _print==True:
                print(f'({now:.1f}/{delayT}) Pressure = {float(dataOut.iloc[-1,2]):.2f} bar    Flow rate = {float(dataOut.iloc[-1,15]):.2f} ul/min        ',end='\r')
            
            now = time.time() - t0

        except KeyboardInterrupt:
            break
    
    return dataOut

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
    dataLead = delayCollect(ADC,delayT=5,_delayTolerance=sampleT)

    PR.set_P(pressure)

    dataMain = delayCollect(ADC,delayT=testT,_delayTolerance=sampleT)
    ADC.stop()
    
    PR.set_P(-1)

    dataFrame = pd.concat([dataLead,dataMain],ignore_index=True)
    dataFrame.columns = ['Pressure (bar)','Flow rate (ul/min)','Time']

    print(f'\nStep pressure change results')
    print(f'----------------------------')
    print(dataFrame.describe())

    dataFrame.loc[:,['Time','Pressure (bar)']].to_csv('./StepDataPressure.csv',index=False,header=True)
    dataFrame.loc[:,['Time','Flow rate (ul/min)']].to_csv('./StepDataFlow.csv',index=False,header=True)

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
    PR.set_P(pressure)
    time.sleep(10)

    dataMain = delayCollect(ADC,delayT=testT,_delayTolerance=sampleT)
    ADC.stop()
    
    PR.set_P(-1)

    dataFrame = dataMain
    dataFrame.columns = ['Pressure (bar)','Flow rate (ul/min)','Time']

    print(f'\Hold pressure change results')
    print(f'----------------------------')
    print(dataFrame.describe(exclude=['Time']))

    dataFrame.loc[:,['Time','Pressure (bar)']].to_csv('./HoldDataPressure.csv',index=False,header=True)
    dataFrame.loc[:,['Time','Flow rate (ul/min)']].to_csv('./HoldDataFlow.csv',index=False,header=True)

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
    dataFrame.columns = ['Pressure (bar)','Flow rate (ul/min)','Time']

    print(f'\nRamp pressure change results')
    print(f'----------------------------')
    print(dataFrame.describe())

    dataFrame.loc[:,['Time','Pressure (bar)']].to_csv('./RampDataPressure.csv',index=False,header=True)
    dataFrame.loc[:,['Time','Flow rate (ul/min)']].to_csv('./RampDataFlow.csv',index=False,header=True)

    # with open('./RampDataPressure.csv') as f:
    #     plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Ramp response - Pressure')
    # with open('./RampDataFlow.csv') as f:
    #     plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Ramp response - Flow rate')

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
    dataLead = delayCollect(ADC,delayT=5)

    PR.set_P(pressure)
    dataPulse = delayCollect(ADC,delayT=0.5)
    PR.set_P(-1)

    dataMain = delayCollect(ADC,delayT=testT)
    ADC.stop()
    
    PR.set_P(-1)

    dataFrame = pd.concat([dataLead,dataPulse,dataMain],ignore_index=True)
    dataFrame.columns = ['Pressure (bar)','Flow rate (ul/min)','Time']

    print(f'\nImpulse pressure change results')
    print(f'----------------------------')
    print(dataFrame.describe(exclude=['Time']))

    dataFrame.loc[:,['Time','Pressure (bar)']].to_csv('./ImpulseDataPressure.csv',index=False,header=True)
    dataFrame.loc[:,['Time','Flow rate (ul/min)']].to_csv('./ImpulseDataFlow.csv',index=False,header=True)

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
    # main()
    pass