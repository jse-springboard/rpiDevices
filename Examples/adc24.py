from rpiDevices.rpi import adc24
import time

def block_method(adc):
    '''
    Demonstration of block method.
    '''
    t0 = time.time()
    
    # Output data from all input channels.
    data_output, data_times = adc.collect(2, method='block')

    drop = time.time() - t0
    print(f'Data from block method (t = {drop:.3f} s):')
    print(f'------------------------------------------')

    for i in adc.channel:
        print(f'Channel {i}  \t|  Time: {data_times[i]}  \t|  Data: {data_output[i]}')
    print('')

def window_method(adc):
    '''
    Demonstration of window method.
    '''
    t0 = time.time()
    # Output data from all input channels. Stop stream after.
    data_output, data_times = adc.collect(2, method='window')

    drop = time.time() - t0
    
    adc.stop()

    print(f'Data from window method (t = {drop:.3f} s):')
    print(f'-------------------------------------------')

    for i in adc.channel:
        print(f'Channel {i}  \t|  Time: {data_times[i]}  \t|  Data: {data_output[i]}')
    print('')

def stream_method(adc):
    '''
    Demonstration of stream method.
    '''
    t0 = time.time()
    
    # Output data from all input channels. Stop stream after.
    data_output, data_times = adc.collect(2, method='stream')
    drop = time.time() - t0
    
    adc.stop()
    
    print(f'Data from stream method (t = {drop:.3f} s):')
    print(f'-------------------------------------------')

    for i in adc.channel:
        print(f'Channel {i}  \t|  Time: {data_times[i]}  \t|  Data: {data_output[i]}')
    print('')

def main():
    '''
    Main code
    '''
    # Start adc instance and initiate system
    # Argument to adc24 class call specifies the channel number and the conversion coefficients [x0, x1] for the ADC output ( y = (ADC_out / ADC_max)*x1 + x0 )
    adc = adc24(channel={1:[0,10],2:[-0.1,300]},vrange={1:0,2:1})
    
    # Add channel to device
    adc.addCh([3,4],vrange={3:2,4:4})

    # Remove channel from device
    adc.rmCh([1])

    # Print coefficients on each channel
    adc.printCoeffs()
    print('')

    # Demonstrate block method
    block_method(adc)

    # Demonstrate window method
    window_method(adc)

    # Demonstrate stream method
    stream_method(adc)

    # Reset device
    adc.reset()

    # Shutdown device
    adc.shutdown()

    # Reset device using modCh command
    adc.modCh({1:[10,0.25],2:[2,3],4:[]})

    adc.shutdown()
    return adc

if __name__ == '__main__':
    adc = main()