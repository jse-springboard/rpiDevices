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
import time, os
import datetime
import pandas as pd

def step(PR,ADC,pressure=2.0,testT=5.0,sampleT=0.5,ID='Step'):
    '''
    Record response to a step change in pressure
    --------------------------------------------
    MUST PASS A vppr() AND adc24() INSTANCE!
    DEFAULTS
    --------
    pressure    -> 2 bar
    testT       -> 5 s
    sampleT     -> 0.2 s
    '''
    print(f'[STEP] Run lead-in for {5} seconds')
    dataLead = ADC.bufferCollect(delayT=5)

    print(f'[VPPR] Set hold pressure to {pressure} bar')
    PR.set_P(pressure)

    print(f'[STEP] Run test for {testT} seconds')
    dataMain = ADC.bufferCollect(delayT=testT)
    ADC.stop()
    
    print(f'[VPPR] Reset pressure regulator')
    PR.set_P(-1)

    dataFrame = pd.concat([dataLead,dataMain],ignore_index=True)

    dataInterval = int(round(sampleT/(dataFrame['Time'].max() / dataFrame.count()[0])))

    if dataInterval < 1:
        dataInterval = 1
        print(f'[DBUG] Data interval set to highest resolution')
    else:    
        print(f'[DBUG] Data interval set to {dataInterval}')

    dataFrame = dataFrame.rolling(window=dataInterval,center=True).mean().dropna().iloc[0::dataInterval,:].reset_index(drop=True)
    
    dataFrame.columns = ['Time','Pressure (bar)','Flow rate (ul/min)']

    print(f'\n{ID} pressure change results')
    print(f'----------------------------')
    print(dataFrame.loc[:,['Pressure (bar)','Flow rate (ul/min)']].describe())

    dataFrame.loc[:,['Time','Pressure (bar)']].to_csv(f'./Data/pressureResponse/{ID}DataPressure.csv',index=False,header=True)
    dataFrame.loc[:,['Time','Flow rate (ul/min)']].to_csv(f'./Data/pressureResponse/{ID}DataFlow.csv',index=False,header=True)
    os.system(f'cd Data/pressureResponse && git add -A && git commit -m "{ID}" && git push')

    return dataFrame

def hold(PR,ADC,pressure=2.0,testT=5.0,sampleT=0.5,_settlingTime = 10,ID='Hold'):
    '''
    Record change in pressure and flow rate over a period of time
    -------------------------------------------------------------
    MUST PASS A vppr() AND adc24() INSTANCE!
    DEFAULTS
    --------
    pressure    -> 2 bar
    testT       -> 5 s
    sampleT     -> 0.2 s
    '''
    
    print(f'[VPPR] Set hold pressure to {pressure} bar')
    PR.set_P(pressure)

    print(f'[{datetime.datetime.now()}] Settling time ({_settlingTime} s)')
    time.sleep(_settlingTime)
    
    print(f'[HOLD] Run test for {testT} seconds')
    dataMain = ADC.bufferCollect(delayT=testT)
    ADC.stop()
    
    print(f'[VPPR] Reset pressure regulator')
    PR.set_P(-1)

    dataInterval = int(round(sampleT/(dataMain['Time'].max() / dataMain.count()[0])))
    print(f'[DBUG] Data interval set to {dataInterval}')

    if dataInterval <= 0:
        dataInterval = 1

    dataFrame = dataMain.rolling(window=dataInterval,center=True).mean().dropna().iloc[0::dataInterval,:].reset_index(drop=True)
    dataFrame.columns = ['Time','Pressure (bar)','Flow rate (ul/min)']

    print(f'\nHold pressure change results')
    print(f'----------------------------')
    print(dataFrame.loc[:,['Pressure (bar)','Flow rate (ul/min)']].describe())

    dataFrame.loc[:,['Time','Pressure (bar)']].to_csv(f'./Data/pressureResponse/{ID}DataPressure.csv',index=False,header=True)
    dataFrame.loc[:,['Time','Flow rate (ul/min)']].to_csv(f'./Data/pressureResponse/{ID}DataFlow.csv',index=False,header=True)
    os.system(f'cd Data/pressureResponse && git add -A && git commit -m "{ID}" && git push')

    return dataFrame

def ramp(PR,ADC,pressure=2.0,rampT=5.0,testT=5.0,sampleT=0.2,ID='Ramp'):
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

    dataFrame.loc[:,['Time','Pressure (bar)']].to_csv('./{ID}DataPressure.csv',index=False,header=True)
    dataFrame.loc[:,['Time','Flow rate (ul/min)']].to_csv('./{ID}DataFlow.csv',index=False,header=True)

    # with open('./RampDataPressure.csv') as f:
    #     plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Ramp response - Pressure')
    # with open('./RampDataFlow.csv') as f:
    #     plot_scatter(f=f,xs='',ys='',size=20,pch='x',colour='white',title='Ramp response - Flow rate')

    return dataFrame

def impulse(PR,ADC,pressure=5.0,testT=5.0,ID='Impulse'):
    '''
    Record response to pressure impulse
    -----------------------------------

    DEFAULTS
    --------
    pressure    -> 5 bar
    testT       -> 5 s
    '''
    print(f'[IMPL] Run lead-in for {5} seconds')
    dataLead = ADC.bufferCollect(delayT=5)

    print(f'[VPPR] Set hold pressure to {pressure} bar')
    PR.set_P(pressure)
    dataPulse = ADC.bufferCollect(delayT=0.5)
    
    print(f'[VPPR] Reset pressure regulator')
    PR.set_P(-1)
    time.sleep(0.5)
    
    print(f'[IMPL] Run impulse response for {testT} seconds')
    dataMain = ADC.bufferCollect(delayT=testT)
    ADC.stop()
    
    PR.set_P(-1)

    dataFrame = pd.concat([dataLead,dataPulse,dataMain],ignore_index=True)

    print(f'[DBUG] Data interval set to HIGH_RES to capture impulse response')
    dataFrame.columns = ['Time','Pressure (bar)','Flow rate (ul/min)']

    print(f'\nImpulse pressure change results')
    print(f'----------------------------')
    print(dataFrame.loc[:,['Pressure (bar)','Flow rate (ul/min)']].describe())

    dataFrame.loc[:,['Time','Pressure (bar)']].to_csv(f'./Data/pressureResponse/{ID}DataPressure.csv',index=False,header=True)
    dataFrame.loc[:,['Time','Flow rate (ul/min)']].to_csv(f'./Data/pressureResponse/{ID}DataFlow.csv',index=False,header=True)
    os.system(f'cd Data/pressureResponse && git add -A && git commit -m "{ID}" && git push')

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