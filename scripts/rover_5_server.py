#! /usr/bin/env python

import serial
import time
import multiprocessing
import sys
import subprocess
import RPIO
import RPIO.PWM
import picam

BLUETOOTH_SERIAL_PORT = "/dev/ttyAMA0"
BLUETOOTH_SERIAL_BAUD_RATE = 9600
MINI_DRIVER_SERIAL_PORT = "/dev/ttyUSB0"
MINI_DRIVER_SERIAL_BAUD_RATE = 9600

PWM_FREQUENCY = 50    # Hz

PWM_DMA_CHANNEL = 0
PWM_SUBCYLCLE_TIME_US = 1000/PWM_FREQUENCY * 1000
PWM_PULSE_INCREMENT_US = 10

PAN_PWM_PIN = 23
TILT_PWM_PIN = 18

#---------------------------------------------------------------------------------------------------
class ServoPWM:
    
    ABSOLUTE_MIN_PULSE_WIDTH_US = 500
    ABSOLUTE_MAX_PULSE_WIDTH_US = 2500
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, pwmPin, minAnglePulseWidthPair, 
        midAnglePulseWidthPair, maxAnglePulseWidthPair ):
        
        # Check that the given angles are valid
        assert( minAnglePulseWidthPair[ 0 ] >= 0 )
        assert( midAnglePulseWidthPair[ 0 ] > minAnglePulseWidthPair[ 0 ] )
        assert( midAnglePulseWidthPair[ 0 ] < maxAnglePulseWidthPair[ 0 ] )
        assert( maxAnglePulseWidthPair[ 0 ] <= 180 )
        
        self.pwmPin = pwmPin
        self.minAnglePulseWidthPair = minAnglePulseWidthPair
        self.midAnglePulseWidthPair = midAnglePulseWidthPair
        self.maxAnglePulseWidthPair = maxAnglePulseWidthPair
        self.lastPulseWidthSet = None
    
    #-----------------------------------------------------------------------------------------------
    def setCommand( self, command ):
        
        # Work out whether the command is an angle, or a pulse width
        if command >= self.ABSOLUTE_MIN_PULSE_WIDTH_US:
            self.setPulseWidth( command )
        else:
            self.setAngle( command )
    
    #-----------------------------------------------------------------------------------------------
    def setPulseWidth( self, pulseWidth ):
        
        # Constrain the pulse width
        if pulseWidth < self.ABSOLUTE_MIN_PULSE_WIDTH_US:
            pulseWidth = self.ABSOLUTE_MIN_PULSE_WIDTH_US
        if pulseWidth > self.ABSOLUTE_MAX_PULSE_WIDTH_US:
            pulseWidth = self.ABSOLUTE_MAX_PULSE_WIDTH_US
        
        # Ensure that the pulse width is an integer multiple of the smallest 
        # possible pulse increment
        pulseIncrementUS = RPIO.PWM.get_pulse_incr_us()
        numPulsesNeeded = int( pulseWidth/pulseIncrementUS )
        pulseWidth = numPulsesNeeded * pulseIncrementUS
    
        if pulseWidth != self.lastPulseWidthSet:
        
            RPIO.PWM.add_channel_pulse( PWM_DMA_CHANNEL, self.pwmPin, 0, numPulsesNeeded )
            self.lastPulseWidthSet = pulseWidth
    
    #-----------------------------------------------------------------------------------------------
    def setAngle( self, angle ):
        
        # Constrain the angle
        if angle < self.minAnglePulseWidthPair[ 0 ]:
            angle = self.minAnglePulseWidthPair[ 0 ]
        if angle > self.maxAnglePulseWidthPair[ 0 ]:
            angle = self.maxAnglePulseWidthPair[ 0 ]
            
        # Convert the angle to a pulse width using linear interpolation
        if angle < self.midAnglePulseWidthPair[ 0 ]:
            
            angleDiff = self.midAnglePulseWidthPair[ 0 ] - self.minAnglePulseWidthPair[ 0 ]
            startPulseWidth = self.minAnglePulseWidthPair[ 1 ]
            pulseWidthDiff = self.midAnglePulseWidthPair[ 1 ] - self.minAnglePulseWidthPair[ 1 ]
            
            interpolation = float( angle - self.minAnglePulseWidthPair[ 0 ] ) / angleDiff
            
            pulseWidth = startPulseWidth + interpolation*pulseWidthDiff
            
        else:
            
            angleDiff = self.maxAnglePulseWidthPair[ 0 ] - self.midAnglePulseWidthPair[ 0 ]
            startPulseWidth = self.midAnglePulseWidthPair[ 1 ]
            pulseWidthDiff = self.maxAnglePulseWidthPair[ 1 ] - self.midAnglePulseWidthPair[ 1 ]
            
            interpolation = float( angle - self.midAnglePulseWidthPair[ 0 ] ) / angleDiff
            
            pulseWidth = startPulseWidth + interpolation*pulseWidthDiff
        
        print "Converted angle {0} to pulse width {1}".format( angle, pulseWidth )
        
        # Now set the pulse width
        self.setPulseWidth( pulseWidth )

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
class ArtifactDetector( ProcessBase ):
    
    IDEAL_LOOP_TIME = 1.0/20.0  # Runs at 20Hz
    IMAGE_WIDTH = 320
    IMAGE_HEIGHT = 240
    IMAGE_JPEG_QUALITY = 85
    HORIZONTAL_FOV_DEGREES = 53.13
    
    MARKER_ID_TO_ARTIFACT_ID_MAP = {
        6 : 3,
        354 : 4,
        724 : 5,
        1014 : 6
    }
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, commandQueue, detectionResultQueue ):
        
        ProcessBase.__init__( self )
        
        self.commandQueue = commandQueue
        self.detectionResultQueue = detectionResultQueue
        self.curDetectionId = 0
        
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        while not self.stopped():
            
            loopStartTime = time.time()
            
            # Get the next command off the top of the queue if there is one
            if not self.commandQueue.empty():
                
                newCommand = self.commandQueue.get_nowait()
                
                # The command is always to detect an artifact so capture an image
                print "Starting detection"
                subprocess.call( [ "raspistill", "-w", "{0}".format( self.IMAGE_WIDTH ), 
                "-h", "240", "-hf", "-vf", "-o", "/home/pi/artifact_detect.jpg" ] )

                print "Photo taken"

                # Try to detect AR markers in the image
                detectionResult = subprocess.check_output( 
                    [ "sudo", "/home/pi/dev/extra/aruco-1.2.4/build/utils/aruco_simple_no_window",
                    "/home/pi/artifact_detect.jpg" ], stderr=subprocess.PIPE )

                print "Detection run"
                print detectionResult
                    
                markerId = -1

                lines = detectionResult.split( "\n" )
                for line in lines:

                    try:
                        parts = line.split( "=" )
                        if len( parts ) >= 2:
                            
                            markerId = int( parts[ 0 ] )    # May raise parse exception
                            
                            coords = parts[ 1 ].split( " " )
                            
                            if len( coords ) >= 4:
                                
                                accX = 0.0
                                numCoordsParsed = 0
                                
                                for coordIdx in range( 4 ):
                                    
                                    xy = coords[ coordIdx ].split( "," )
                                    if len( xy ) >= 1:
                                        
                                        x = float( xy[ 0 ].strip( "(" ) )
                                        accX += x
                                        numCoordsParsed += 1
                                        
                                if numCoordsParsed == 4:
                                    
                                    markerCentreX = accX/4.0
                                    break                   # Don't bother to parse other lines
                    except:
                        pass    # Ignore errors that occur when parsing the line

                artifactFound = False
                if markerId in self.MARKER_ID_TO_ARTIFACT_ID_MAP:
                    
                    artifactId = self.MARKER_ID_TO_ARTIFACT_ID_MAP[ markerId ]
                    artifactFound = True
                        
                if not artifactFound:
                    artifactId = -1
                    markerCentreX = self.IMAGE_WIDTH/2.0

                markerHeadingDegrees = (markerCentreX - self.IMAGE_WIDTH/2.0)/(self.IMAGE_WIDTH/2.0) * (self.HORIZONTAL_FOV_DEGREES/2.0)
        
                result = ( self.curDetectionId, artifactId, markerHeadingDegrees )
                self.detectionResultQueue.put( result )
                self.curDetectionId += 1
                
            # Sleep if needed
            elapsedTime = time.time() - loopStartTime
            loopTimeRemaining = self.IDEAL_LOOP_TIME - elapsedTime
            
            if loopTimeRemaining > 0.0:
                time.sleep( loopTimeRemaining )
        
#---------------------------------------------------------------------------------------------------
class StateReporter( ProcessBase ):

    """Reads from motor driver and transmits status"""
    
    MAX_SONAR_DISTANCE_CM = 400
    SONAR_PIN = 24
    SONAR_TIMEOUT = 1.0
    IDEAL_LOOP_TIME = 1.0/20.0  # Runs at 20Hz
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, bluetoothSerial, miniDriverSerial, detectionResultQueue ):
        
        ProcessBase.__init__( self )
        
        self.bluetoothSerial = bluetoothSerial
        self.miniDriverSerial = miniDriverSerial
        self.detectionResultQueue = detectionResultQueue
        
        self.leftEncoderCount = 0
        self.rightEncoderCount = 0
        self.sonarDistanceCM = self.MAX_SONAR_DISTANCE_CM
        self.serialBuffer = ""
        self.lastDetectionId = -1
        self.detectedArtifactId = -1
        self.detectedArtifactHeadingDegrees = 0.0
    
    #---------------------------------------------------------------------------------------------------
    def readDistanceFromSonarCM( self ):
        
        distance = 0.0
        
        # Send the start signal
        RPIO.setup( self.SONAR_PIN, RPIO.OUT )
        RPIO.output( self.SONAR_PIN, RPIO.LOW )
        #time.sleep( 0.00002 )
        
        RPIO.output( self.SONAR_PIN, RPIO.HIGH )
        time.sleep( 0.00001 )
        RPIO.output( self.SONAR_PIN, RPIO.LOW )
        
        # Prepare to receive the response
        RPIO.setup( self.SONAR_PIN, RPIO.IN, pull_up_down=RPIO.PUD_DOWN )
        
        # TODO: It would be nicer to use interrupts here, but I can't get them to work just yet...  
        waitStartTime = time.time()
        pulseStartTime = time.time()
        while RPIO.input( self.SONAR_PIN ) == 0:
            curTime = time.time()
            pulseStartTime = curTime
            
            if curTime - waitStartTime > self.SONAR_TIMEOUT:
                return self.MAX_SONAR_DISTANCE_CM

        pulseEndTime = curTime = time.time()
        while RPIO.input( self.SONAR_PIN ) == 1:
            curTime = time.time()
            pulseEndTime = curTime
            
            if curTime - waitStartTime > self.SONAR_TIMEOUT:
                return self.MAX_SONAR_DISTANCE_CM
        
        measuredTimeUS = (pulseEndTime - pulseStartTime)*1000000.0
        distanceCM = int( measuredTimeUS / 58.0 )   # Using formula on http://www.seeedstudio.com/wiki/Ultra_Sonic_range_measurement_module
        
        if distanceCM > self.MAX_SONAR_DISTANCE_CM:
            distanceCM = self.MAX_SONAR_DISTANCE_CM
        
        return distanceCM
    
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
                    except:
                        pass    # Ignore parsing errors
            
            # Read from the sonar sensor
            self.sonarDistanceCM = self.readDistanceFromSonarCM()
            
            # Get the latest detection result if it exists
            if not self.detectionResultQueue.empty():
                
                detectionResult = self.detectionResultQueue.get_nowait()
                self.lastDetectionId, self.detectedArtifactId, self.detectedArtifactHeadingDegrees = detectionResult
            
            # Transmit the current status back
            #print "Writing", "{0} {1}\n".format( self.leftEncoderCount, self.rightEncoderCount )
            self.bluetoothSerial.write( "{0} {1} {2} {3} {4} {5}\n".format( 
                self.leftEncoderCount, self.rightEncoderCount, self.sonarDistanceCM,
                self.lastDetectionId, self.detectedArtifactId, self.detectedArtifactHeadingDegrees ) )
    
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
    
    BUZZER_PIN = 8
    BUZZER_TIME = 0.25
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, bluetoothSerial, miniDriverSerial, commandQueue ):
        
        ProcessBase.__init__( self )
        
        self.bluetoothSerial = bluetoothSerial
        self.miniDriverSerial = miniDriverSerial
        self.commandQueue = commandQueue
        
        self.buzzerActive = False
        self.buzzerStartTime = 0
        RPIO.setup( self.BUZZER_PIN, RPIO.OUT )
        
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
                elif command == "u":
                    
                    self.buzzerActive = True
                    self.buzzerStartTime = time.time()
                elif command == "p":
                    
                    # Try to detect an artifact
                    self.commandQueue.put( command )
                
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
            
            # Update the buzzer
            if self.buzzerActive: 
                if time.time() - self.buzzerStartTime > self.BUZZER_TIME:
                    self.buzzerActive = False
                    
            if self.buzzerActive:
                
                # Buzzer on
                RPIO.output( self.BUZZER_PIN, RPIO.HIGH )
                
            else:
                
                # Buzzer off
                RPIO.output( self.BUZZER_PIN, RPIO.LOW )
            
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
    
    # Create ServoPWM instances to control the servos
    #panServoPWM = ServoPWM( PAN_PWM_PIN, 
        #minAnglePulseWidthPair=( 45.0, 1750 ), 
        #midAnglePulseWidthPair=( 90.0, 1200 ), 
        #maxAnglePulseWidthPair=( 135.0, 700.0 ) )
    #tiltServoPWM = ServoPWM( TILT_PWM_PIN, 
        #minAnglePulseWidthPair=( 45.0, 1850 ), 
        #midAnglePulseWidthPair=( 90.0, 1500 ), 
        #maxAnglePulseWidthPair=( 180.0, 500.0 ) )

    ## Setup RPIO, and prepare for PWM signals
    RPIO.setmode( RPIO.BCM )

    #RPIO.PWM.setup( pulse_incr_us=PWM_PULSE_INCREMENT_US )
    #RPIO.PWM.init_channel( PWM_DMA_CHANNEL, PWM_SUBCYLCLE_TIME_US )
    
    #panServoPWM.setAngle( 90.0 )
    #tiltServoPWM.setAngle( 90.0 )
    
    bluetoothSerial = serial.Serial( BLUETOOTH_SERIAL_PORT, baudrate=BLUETOOTH_SERIAL_BAUD_RATE, timeout=0 )
    miniDriverSerial = serial.Serial( MINI_DRIVER_SERIAL_PORT, baudrate=MINI_DRIVER_SERIAL_BAUD_RATE, timeout=0 )

    # Wait until we hear from the mini driver
    print "Waiting for mini driver..."
    time.sleep( 12.0 )
    
    while miniDriverSerial.inWaiting() == 0:
        
        pass
    
    # Create processes
    print "Starting up..."
    
    commandQueue = multiprocessing.Queue()
    detectionResultQueue = multiprocessing.Queue()
    
    artifactDetector = ArtifactDetector( commandQueue, detectionResultQueue )
    stateReporter = StateReporter( bluetoothSerial, miniDriverSerial, detectionResultQueue )
    commandHandler = CommandHandler( bluetoothSerial, miniDriverSerial, commandQueue )

    artifactDetector.start()
    stateReporter.start()
    commandHandler.start()
    
    while True:
        
        # Wait for a ctrl+c
        try:
            time.sleep( 0.1 )
            
        except KeyboardInterrupt:
            cleanupProcesses( ( artifactDetector, stateReporter, commandHandler ) )
            
            #RPIO.PWM.clear_channel_gpio( PWM_DMA_CHANNEL, panServoPWM.pwmPin )
            #RPIO.PWM.clear_channel_gpio( PWM_DMA_CHANNEL, tiltServoPWM.pwmPin )
            
            #RPIO.PWM.cleanup()
            RPIO.cleanup()
            sys.exit()

                
