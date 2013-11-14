#! /usr/bin/env python

import serial
import time
import multiprocessing
import sys

BLUETOOTH_SERIAL_PORT = "/dev/ttyAMA0"
BLUETOOTH_SERIAL_BAUD_RATE = 9600
MINI_DRIVER_SERIAL_PORT = "/dev/ttyUSB0"
MINI_DRIVER_SERIAL_BAUD_RATE = 9600

#---------------------------------------------------------------------------------------------------
class ProcessBase( multiprocessing.Process ):
    def __init__( self ):
        multiprocessing.Process.__init__( self )
        self._stop = multiprocessing.Event()

    def stop( self ):
        self._stop.set()

    def stopped( self ):
        return self._stop.is_set()
    
#---------------------------------------------------------------------------------------------------
class StateReporter( ProcessBase ):

    """Reads from motor driver and transmits status"""
    
    MAX_SONAR_DISTANCE = 4.0
    IDEAL_LOOP_TIME = 1.0/20.0  # Runs at 20Hz
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, bluetoothSerial, miniDriverSerial ):
        
        ProcessBase.__init__( self )
        
        self.bluetoothSerial = bluetoothSerial
        self.miniDriverSerial = miniDriverSerial
        
        self.leftEncoderCount = 0
        self.rightEncoderCount = 0
        self.sonarDistance = self.MAX_SONAR_DISTANCE
        self.serialBuffer = ""
        
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        while not self.stopped():
            
            loopStartTime = time.time()
            
            # Read status messages from the mini driver
            numBytesAvailable = self.miniDriverSerial.inWaiting()
            if numBytesAvailable > 0:
                
                self.serialBuffer += self.miniDriverSerial.read( numBytesAvailable )
                
                statusData = ""
                endOfLinePos = self.serialBuffer.find( "\n" )
                
                while endOfLinePos != -1:
                    
                    # Remove lines found in the serial data
                    statusData = self.serialBuffer[ :endOfLinePos ]
                    self.serialBuffer = self.serialBuffer[ endOfLinePos + 1: ]
                    
                    endOfLinePos = self.serialBuffer.find( "\n" )
                    
                # Extract the current status of the robot from the last line                
                statusItems = statusData.split()
                if len( statusItems ) >= 2:
                    
                    try:
                        self.leftEncoderCount = int( statusItems[ 0 ] )
                        self.rightEncoderCount = int( statusItems[ 1 ] )
                        #self.sonarDistance = int( statusItems[ 2 ] )
                    except:
                        pass    # Ignore parsing errors
            
            # Transmit the current status back
            #print "Writing", "{0} {1}\n".format( self.leftEncoderCount, self.rightEncoderCount )
            self.bluetoothSerial.write( "{0} {1}\n".format( self.leftEncoderCount, self.rightEncoderCount ) )
    
            # Sleep if needed
            elapsedTime = time.time() - loopStartTime
            loopTimeRemaining = self.IDEAL_LOOP_TIME - elapsedTime
            
            if loopTimeRemaining > 0.0:
                time.sleep( loopTimeRemaining )
    
#---------------------------------------------------------------------------------------------------
class CommandHandler( ProcessBase ):

    """Reads from bluetoothSerial and then controls the robot"""
    
    IDEAL_LOOP_TIME = 1.0/20.0  # Runs at 20Hz
    
    COMMAND_STATE_DICT = {
        "l" : "TurningLeft",
        "r" : "TurningRight",
        "f" : "DrivingForward",
        "b" : "DrivingBackward",
        "s" : "Stopped"
    }
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, bluetoothSerial, miniDriverSerial ):
        
        ProcessBase.__init__( self )
        
        self.bluetoothSerial = bluetoothSerial
        self.miniDriverSerial = miniDriverSerial
        
        self.roverState = "Stopped"
        
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        while not self.stopped():
            
            loopStartTime = time.time()
            
            # Read commands from bluetooth
            while self.bluetoothSerial.inWaiting():
                
                command = self.bluetoothSerial.read()
                if command in self.COMMAND_STATE_DICT:
                    self.roverState = self.COMMAND_STATE_DICT[ command ]
                
            # Use the current rover state to work out what to send to the mini driver
            if self.roverState == "Stopped":
                
                self.miniDriverSerial.write( "s" )
                
            elif self.roverState == "TurningLeft":
                
                self.miniDriverSerial.write( "l" )
                
            elif self.roverState == "TurningRight":
                
                self.miniDriverSerial.write( "r" )
            
            elif self.roverState == "DrivingForward":
                
                self.miniDriverSerial.write( "f" )
                
            elif self.roverState == "DrivingBackward":
                
                self.miniDriverSerial.write( "b" )
                
            else:
                
                # Should never get here, but switch back to stopped state if we do
                self.roverState = "Stopped"
                self.miniDriverSerial.write( "s" )
            
            # Sleep if needed
            elapsedTime = time.time() - loopStartTime
            loopTimeRemaining = self.IDEAL_LOOP_TIME - elapsedTime
            
            if loopTimeRemaining > 0.0:
                time.sleep( loopTimeRemaining )

#---------------------------------------------------------------------------------------------------
def cleanupProcesses( processes ):
    for process in processes:
        process.stop()

    for process in processes:
        process.join()
                
#---------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    
    bluetoothSerial = serial.Serial( BLUETOOTH_SERIAL_PORT, baudrate=BLUETOOTH_SERIAL_BAUD_RATE, timeout=0 )
    miniDriverSerial = serial.Serial( MINI_DRIVER_SERIAL_PORT, baudrate=MINI_DRIVER_SERIAL_BAUD_RATE, timeout=0 )

    # Wait until we hear from the mini driver
    print "Waiting for mini driver..."
    time.sleep( 12.0 )
    
    while miniDriverSerial.inWaiting() == 0:
        
        pass
    
    # Create processes
    print "Starting up..."
    
    stateReporter = StateReporter( bluetoothSerial, miniDriverSerial )
    commandHandler = CommandHandler( bluetoothSerial, miniDriverSerial )

    stateReporter.start()
    commandHandler.start()
    
    while True:
        
        # Wait for a ctrl+c
        try:
            time.sleep( 0.1 )
        except KeyboardInterrupt:
            cleanupProcesses( ( stateReporter, commandHandler ) )
            sys.exit()

                
