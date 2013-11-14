#! /usr/bin/env python

import serial
import time
import multiprocessing
import sys

MINI_DRIVER_SERIAL_PORT = "/dev/ttyUSB0"
MINI_DRIVER_SERIAL_BAUD_RATE = 9600

#---------------------------------------------------------------------------------------------------
miniDriverSerial = serial.Serial( MINI_DRIVER_SERIAL_PORT, baudrate=MINI_DRIVER_SERIAL_BAUD_RATE, timeout=0 )

# Wait for serial connection to open
time.sleep( 5 )
numStatusMessagesReceived = 0

leftEncoderCount = 0
rightEncoderCount = 0

serialBuffer = ""
state = "Waiting"

while True:

    loopStartTime = time.time()
            
    # Read status messages from the rover
    numBytesAvailable = miniDriverSerial.inWaiting()
    if numBytesAvailable > 0:
        
        serialBuffer += miniDriverSerial.read( numBytesAvailable )
        
        statusData = ""
        endOfLinePos = serialBuffer.find( "\n" )
        
        while endOfLinePos != -1:
            
            # Remove lines found in the serial data
            statusData = serialBuffer[ :endOfLinePos ]
            serialBuffer = serialBuffer[ endOfLinePos + 1: ]
            
            endOfLinePos = serialBuffer.find( "\n" )
            
        # Extract the current status of the robot from the last line                
        statusItems = statusData.split()
        if len( statusItems ) >= 2:
            
            try:
                leftEncoderCount = int( statusItems[ 0 ] )
                rightEncoderCount = int( statusItems[ 1 ] )
                
                numStatusMessagesReceived += 1
                #self.sonarDistance = int( statusItems[ 2 ] )
            except:
                pass    # Ignore parsing errors
            

    # Update the state machine
    if state == "Waiting":
        
        if numStatusMessagesReceived > 2:
            
            startEncoderCount = leftEncoderCount
            miniDriverSerial.write( "f" )
            state = "GoingForward"
    
    elif state == "GoingForward":
        
        if leftEncoderCount - startEncoderCount  >= 1000:
            
            miniDriverSerial.write( "s" )
            state = "Stopped"
    
    elif state == "Stopped":
        
        print "Went forward", leftEncoderCount - startEncoderCount
    