#! python3
import serial;
import matplotlib.pyplot as plt
from drawnow import *;
import multiexit;
import string
import csv
from time import strftime

# Writes incoming semicolon separated values (SCSVs) to CSV file in the directory where this file is placed.
# Will write each SCSV printed to the terminal to CSV, including some initial headersset in this file
# MCU output should be adjusted to just include information to be stored.
# Designed for immediate logging, and will not save CSV until the software is closed.
# Recommended only for very short logging periods, but can handle higher frequency logging than the other logger.

# Function that ensures the serial is freed when the program exits.
def doAtExit():
    serialDevice.close()
    log.close() #Close the log and store it at closing!
    
# Register atExitProcedures which ensure the serial is closed at exit
multiexit.install();
multiexit.register(doAtExit)

# Initialise some 'global' variables for storage or control.
values = []; # Array for read values to plot
useInt = 0 # Set whether printing integers or floats. 0-> Prints floats, 1->Prints integers


try:
    serialDevice = serial.Serial('COM3', 115200)   
except serial.SerialException: #exit if the serial was not opened properly
    print ("Could not connect to Serial port, either busy or not connected")
    exit() # Prompts an exit dialogue so that the message can be read and exits afterwards

# Until measurement finished, open and leave log open for rapid writing. 
# The file will only exist in MEMORY until this logger is closed, at which point the file is closed/saved.
# If the program//system/others malfunctions throughout the logging, the file might not be saved.
# The logging can be saved/exited by printing "Complete" from the MCU
fname = 'StabilityLog_{0}.csv'.format(strftime('%Y-%m-%d_%H-%M-%S'))
with open(fname, 'w', newline='') as log:
    log.write('\nLog Started at {0}\n'.format(strftime('%Y-%m-%d %H:%M:%S')))
    log.write("Time,Tick,Temp,V_R1,")
    for i in range(0,128):
        log.write("{0},".format(i))
    log.write("Mean\n")
    while True:

        # Check if data is waiting on the serial, read the data if available
        while (serialDevice.inWaiting()==0):
            pass

        # Grab incoming data, remove exess characters and print to terminal
        valueReadStr = str(serialDevice.readline())[2:-5]
        valueReadStr = valueReadStr.replace(';',',')
        print(valueReadStr); # Print serial data to terminal as well
        if valueReadStr == 'Complete':
            log.close()
            exit()

        # Write Time, Temperature, V_R1, Pixels from serial
        log.write("{0},{1}\n".format(strftime('%Y-%m-%d %H:%M:%S'),valueReadStr))
