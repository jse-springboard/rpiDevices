from Devices.rpi import *

# Start adc instance and initiate system
adc = adc24(channel={1:[0,10],2:[-0.1,300]})

# Output data from all input channels.
data_output, data_times = adc.all_out()

channel1_output = {'Time':data_times[1],'Data':data_output[1]}
channel2_output = {'Time':data_times[2],'Data':data_output[2]}