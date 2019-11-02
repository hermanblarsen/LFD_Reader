#! python3
import serial;
import matplotlib.pyplot as plt
from drawnow import *;
import multiexit;
import string
import csv
from time import strftime, sleep

# Writes incoming semicolon separated values (SCSVs) to CSV file in the directory where this file is placed.
# Will write each SCSV printed to the terminal to CSV, including some initial headersset in this file
# MCU output should be adjusted to just include information to be stored.
# Designed for slow (interval >0.5s) readout over several hours. Can be adjusted by the sleep in the serial check loop.

# Function that ensures the serial is freed when the program exits.
def doAtExit():
    serialDevice.close()
    if not log.closed:
        log.close()
    
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

fname = 'StabilityLogLong_{0}.csv'.format(strftime('%Y-%m-%d_%H-%M-%S'))
#Write initial stuff into the file, aka headings
with open(fname, 'w', newline='') as log:
    log.write('\nLog Started at {0}\n'.format(strftime('%Y-%m-%d %H:%M:%S')))
    log.write("Time,Tick,Temp,V_R1,")
    for i in range(0,128):
        log.write("{0},".format(i))
    log.write("Mean\n")
    log.close()

#Until measurement finished, open log and write each time. Closing the log ensures that it is saved.
# This avoids that the file is only in memory, which means that if the program somehow crashes,
#  the file is not saved.
while True:

    # Check if data is waiting on the serial, read the data if available
    while (serialDevice.inWaiting()==0):
        sleep(0.5) #Added to reduce busyiness in busy wait loop. Not good implementation, but only used for device testing.
        pass

    #Open file for writing:
    log = open(fname, 'a', newline='')
    
    # Grab incoming data, remove exess characters and print to terminal
    #  This expects ';'-separated values from the program, which are then exchanged for ',''s
    valueReadStr = str(serialDevice.readline())[2:-5]
    valueReadStr = valueReadStr.replace(';',',')
    print(valueReadStr); # Print serial data to terminal as well

    # Write Time, Temperature, V_R1, Pixels from serial
    log.write("{0},{1}\n".format(strftime('%Y-%m-%d %H:%M:%S'),valueReadStr))

    #Close the log for saving file
    log.close()
    
