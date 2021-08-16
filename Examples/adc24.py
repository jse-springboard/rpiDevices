from rpiDevices.rpi import adc24

if __name__ == '__main__':
    # Start adc instance and initiate system
    # Argument to adc24 class call specifies the channel number and the conversion coefficients [x0, x1] for the ADC output ( y = (ADC_out / ADC_max)*x1 + x0 )
    adc = adc24(channel={1:[0,10],2:[-0.1,300]})

    # Output data from all input channels.
    data_output, data_times = adc.collect(method='stream',nsamples=10)

    channel1_output = {'Time':data_times[1],'Data':data_output[1]}
    channel2_output = {'Time':data_times[2],'Data':data_output[2]}

    # Shutdown unit
    adc.shutdown()