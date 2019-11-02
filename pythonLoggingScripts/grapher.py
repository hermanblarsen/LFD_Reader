#! python3
import serial;
import matplotlib.pyplot as plt
from drawnow import *;
import multiexit;
import string
from time import sleep

#Graphs incoming semicolon separated values in matplotlib to visually interpret.
# Written to print 128 values, equal to the number of pixels in the TSL1401CL Linear CCD Sensor.
# Plots will be overwritten by blanks if non-plotting values are sent to the serial.

# A function that graphs the values received from the serial
def plotValues():
    plt.title('TSL1401CL Data')
    plt.ylabel('Delta Intensity')
    plt.plot(values, 'rx-', label='values')
    plt.grid(True, 'both', 'both', linestyle='-.')
    plt.xlim(0, 127) #Size of the CCD Sensor

# Function that ensures the serial is freed when the program exits.
def doAtExit():
    serialDevice.close()
    
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
    exit("Exit") # Prompts an exit dialogue so that the message can be read and exits afterwards

drawnow(plotValues,False, False, True) # Draw initial window
while True:
    # Check if data is waiting on the serial, read the data if available
    while (serialDevice.inWaiting()==0):
        plt.pause(1) #Pauses the plot so it can be moved and interacted with without going into "Not Responding"
        pass

    # Grab incoming data, remove exess characters and print to terminal
    valueReadStr = str(serialDevice.readline())[2:-5] 
    print(valueReadStr); # Print serial data to terminal as well

    try: #Split the values between ';' or go back to wait if no ';' exists as no plotting should occur
        splitValues = valueReadStr.split(';')
    except TypeError:
        continue

    # Collate and graph the new values if code reaches here
    values.clear()
    for val in splitValues[2:]:
        try:
            valPlot = int(val) if useInt else float(val)
        except ValueError:
            continue
        values.append(valPlot)
    drawnow(plotValues,False, False, True)
