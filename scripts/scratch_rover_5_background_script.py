#!/usr/bin/python
# This code is copyright Dawn Robotics under GPL v2 12/Nov/2013
# derived from Graham Taylor under GPL v2 1/Sep/2013
# derived from Simon Walters under GPL v2
# derived from scratch_handler by Thomas Preston

import socket
import time
import sys
import errno
import multiprocessing
import math
import serial
import scratch_background

#---------------------------------------------------------------------------------------------------
class RobotController( scratch_background.ScratchBase ):
    
    MIN_TIME_BETWEEN_CONTROL_STEPS = 0.05
    MIN_TIME_BETWEEN_SENSOR_UPDATES = 0.1
    
    ENCODER_TICKS_PER_CM = 1000.0/56.0
    ENCODER_TICKS_PER_DEGREE = 490.0/90.0
    TICK_REDUCTION = 50.0   # This aims to reduce the effect of control delay
    
    MAX_ULTRASONIC_RANGE_CM = 400
    TIME_FOR_BEEP = 0.4
    MAX_TIME_TO_WAIT_FOR_ARTIFACT_DETECTION = 30.0
    MAX_NUM_BEEPS = 9
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, socket, commandQueue, bluetoothSerial ):
        scratch_background.ScratchBase.__init__( self, socket )
        
        self.commandQueue = commandQueue
        self.bluetoothSerial = bluetoothSerial
        self.timeOfLastControlStep = time.time()
        self.timeOfLastSensorUpdate = time.time()
        self.numStatusMessagesReceived = 0
        self.connectedToRover = False
        self.serialBuffer = ""
        self.leftEncoderCount = 0
        self.rightEncoderCount = 0
        self.ultrasonicRangeCM = self.MAX_ULTRASONIC_RANGE_CM
        self.lastDetectionID = -1
        self.artifactID = -1
        self.artifactBearingDegrees = 0.0
        
        self.reset()
    
    #-----------------------------------------------------------------------------------------------
    def parseNumber( self, numStr ):
        
        numberParsed = False
        number = 0
        
        try:
            number = int( numStr )
            numberParsed = True
        except:
            pass
            
        return number
    
    #-----------------------------------------------------------------------------------------------
    def reset( self ):
        
        self.roverX = scratch_background.ROVER_START_X
        self.roverY = scratch_background.ROVER_START_Y
        self.headingDegrees = scratch_background.ROVER_START_HEADING_DEGREES
        self.curCommand = None
        self.allCommandsComplete = False
        
        self.lastRoverXSent = -10000
        self.lastRoverYSent = -10000
        self.lastRoverHeadingDegreesSent = -10000
        self.lastUltrasonicRangeCMSent = -10000
        self.lastArtifactIDSent = -10000
        self.lastArtifactBearingDegreesSent = -10000
        self.lastAllCommandsCompleteSent = -10000
        
        self.distanceToMove = 0
        self.degreesToTurn = 0
    
    #-----------------------------------------------------------------------------------------------
    def getRoverStatusMessages( self ):
        
        oldLeftEncoderCount = self.leftEncoderCount
        oldRightEncoderCount = self.rightEncoderCount
        
        # Check for status messages from the rover
        numBytesAvailable = self.bluetoothSerial.inWaiting()
        if numBytesAvailable > 0:
            
            self.serialBuffer += self.bluetoothSerial.read( numBytesAvailable )
            
            statusData = ""
            endOfLinePos = self.serialBuffer.find( "\n" )
            
            while endOfLinePos != -1:
                
                # Remove lines found in the serial data
                statusData = self.serialBuffer[ :endOfLinePos ]
                self.serialBuffer = self.serialBuffer[ endOfLinePos + 1: ]
                
                endOfLinePos = self.serialBuffer.find( "\n" )
                
            # Extract the current status of the robot from the last line                
            statusItems = statusData.split()
            if len( statusItems ) >= 6:
                
                try:
                    self.leftEncoderCount = int( statusItems[ 0 ] )
                    self.rightEncoderCount = int( statusItems[ 1 ] )
                    self.ultrasonicRangeCM = int( statusItems[ 2 ] )
                    self.lastDetectionID = int( statusItems[ 3 ] )
                    self.artifactID = int( statusItems[ 4 ] )
                    self.artifactBearingDegrees = float( statusItems[ 5 ] )
                    
                    self.numStatusMessagesReceived += 1
                except:
                    pass    # Ignore parsing errors
                    
            # Update the robot position from any change to the status message            
            leftEncoderDiff = self.leftEncoderCount - oldLeftEncoderCount
            rightEncoderDiff = self.rightEncoderCount - oldRightEncoderCount
                
            headingX = math.sin( math.radians( self.headingDegrees ) )
            headingY = math.cos( math.radians( self.headingDegrees ) )
            
            if leftEncoderDiff > 0.0:
                
                if rightEncoderDiff > 0.0:
                    
                    # Going forwards
                    distanceMovedCM = leftEncoderDiff/self.ENCODER_TICKS_PER_CM
                    self.roverX += distanceMovedCM*headingX
                    self.roverY += distanceMovedCM*headingY
                        
                else:
                    
                    # Turning right
                    angleTurnedDegrees = leftEncoderDiff/self.ENCODER_TICKS_PER_DEGREE
                    self.headingDegrees += angleTurnedDegrees
            
            elif leftEncoderDiff < 0.0:
                
                if rightEncoderDiff < 0.0:
                    
                    # Going backwards
                    distanceMovedCM = leftEncoderDiff/self.ENCODER_TICKS_PER_CM
                    self.roverX += distanceMovedCM*headingX
                    self.roverY += distanceMovedCM*headingY
                        
                else:
                    
                    # Turning left
                    angleTurnedDegrees = leftEncoderDiff/self.ENCODER_TICKS_PER_DEGREE
                    self.headingDegrees += angleTurnedDegrees
            
            # Normalise heading
            while self.headingDegrees < 0.0:
                self.headingDegrees += 360.0
            while self.headingDegrees >= 360.0:
                self.headingDegrees -= 360.0
    
    #-----------------------------------------------------------------------------------------------
    def processScratchCommands( self ):
        
        # Get the next command off the top of the queue if there is one
        if not self.commandQueue.empty():
            
            newCommand = self.commandQueue.get_nowait()
            newCommand = newCommand.strip().lower()
            
            if newCommand == "reset":
                
                self.reset()
            
            elif newCommand == "stop":
                
                self.curCommand = None
                self.allCommandsComplete = True
            
            elif newCommand == "prepareforcommand":
                
                if self.allCommandsComplete == True:
                    self.allCommandsComplete = False
            
            else:
                
                if self.allCommandsComplete == True or self.curCommand != None:
                    
                    print "Ignoring command ({0}) as we're not ready for it".format( newCommand )
        
                else:
                    
                    if newCommand.startswith( "move" ):
                        
                        self.curCommand = "move"
                        self.distanceToMove = self.parseNumber( newCommand[ len( "move" ): ] )
                        self.moveStartEncoderCount = self.leftEncoderCount
                        
                    elif newCommand.startswith( "turn" ):
                        
                        self.curCommand = "turn"
                        self.degreesToTurn = self.parseNumber( newCommand[ len( "turn" ): ] )
                        self.turnStartEncoderCount = self.leftEncoderCount
                        
                    elif newCommand.startswith( "beep" ):
                        
                        self.curCommand = "beep"
                        self.numBeepsToMake = self.parseNumber( newCommand[ len( "beep" ): ] )
                        if self.numBeepsToMake > self.MAX_NUM_BEEPS:
                            self.numBeepsToMake = self.MAX_NUM_BEEPS
                        self.numBeepsMade = 0
                        self.waitingForBeepToComplete = False
                        
                    elif newCommand == "detectartifact":
                        
                        self.curCommand = "detectartifact"
                        self.waitingForDetectionToComplete = False
    
    #-----------------------------------------------------------------------------------------------
    def updateControlLoop( self ):
        
        # Update the robot control
        curTime = time.time()
        if curTime - self.timeOfLastControlStep >= self.MIN_TIME_BETWEEN_CONTROL_STEPS:
            
            timeDiff = curTime - self.timeOfLastControlStep
            
            if self.curCommand == "move":
                
                moveComplete = False
                ticksToMove = self.distanceToMove * self.ENCODER_TICKS_PER_CM
                
                if ticksToMove >= 0.0:
                    
                    ticksToMove -= self.TICK_REDUCTION
                    ticksMoved = self.leftEncoderCount - self.moveStartEncoderCount
                    
                    if ticksMoved >= ticksToMove:
                        
                        moveComplete = True
                        
                    else:
                        
                        self.bluetoothSerial.write( "f" )
                        
                else:
                    
                    ticksToMove += self.TICK_REDUCTION
                    ticksMoved = self.leftEncoderCount - self.moveStartEncoderCount
                    
                    if ticksMoved <= ticksToMove:
                        
                        moveComplete = True
                        
                    else:
                        
                        self.bluetoothSerial.write( "b" )
                
                if moveComplete:
                    self.curCommand = None
                    self.allCommandsComplete = True
                    
            elif self.curCommand == "turn":
                
                turnComplete = False
                ticksToTurn = self.degreesToTurn * self.ENCODER_TICKS_PER_DEGREE
                
                if ticksToTurn >= 0.0:
                    
                    ticksToTurn -= self.TICK_REDUCTION
                    ticksTurned = self.leftEncoderCount - self.turnStartEncoderCount
                    
                    if ticksTurned >= ticksToTurn:
                        
                        turnComplete = True
                        
                    else:
                        
                        self.bluetoothSerial.write( "r" )
                        
                else:
                    
                    ticksToTurn += self.TICK_REDUCTION
                    ticksTurned = self.leftEncoderCount - self.turnStartEncoderCount
                    
                    if ticksTurned <= ticksToTurn:
                        
                        turnComplete = True
                        
                    else:
                        
                        self.bluetoothSerial.write( "l" )
                
                if turnComplete:
                    self.curCommand = None
                    self.allCommandsComplete = True
            
            elif self.curCommand == "beep":
                
                if self.waitingForBeepToComplete \
                    and curTime - self.timeBeepStarted >= self.TIME_FOR_BEEP:
                    
                    self.numBeepsMade += 1
                    self.waitingForBeepToComplete = False
                    
                if self.numBeepsMade >= self.numBeepsToMake:
                    
                    self.curCommand = None
                    self.allCommandsComplete = True
                    
                else:
                    
                    if not self.waitingForBeepToComplete:
                        # Send a beep command
                        self.bluetoothSerial.write( "u" )
                        self.waitingForBeepToComplete = True
                        self.timeBeepStarted = time.time()
                        
            elif self.curCommand == "detectartifact":

                if self.waitingForDetectionToComplete:
                    
                    if curTime - self.timeDetectionStarted > self.MAX_TIME_TO_WAIT_FOR_ARTIFACT_DETECTION \
                        or self.lastDetectionID != self.waitStartDetectionID:
                            
                        # We either time out or see that the detection has finished
                        self.curCommand = None
                        self.allCommandsComplete = True
                        
                else:
                    
                    # Send a detection command
                    self.waitStartDetectionID = self.lastDetectionID
                    self.bluetoothSerial.write( "p" )
                    self.waitingForDetectionToComplete = True
                    self.timeDetectionStarted = time.time()
                
            else:
                
                self.bluetoothSerial.write( "s" )
            
            self.timeOfLastControlStep = curTime
    
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        while not self.stopped():

            
            self.getRoverStatusMessages()
            
            if self.numStatusMessagesReceived > 2 and not self.connectedToRover:
            
                self.connectedToRover = True
                print "Connected to Rover"
        
        
            if self.connectedToRover:
        
                self.processScratchCommands()
                self.updateControlLoop()
                
            
            # Send current state back to scratch
            curTime = time.time()
            if curTime - self.timeOfLastSensorUpdate >= self.MIN_TIME_BETWEEN_SENSOR_UPDATES:
                
                if self.roverX != self.lastRoverXSent:
                    self.sendSensorUpdate( "roverX", self.roverX )
                    self.lastRoverXSent = self.roverX
                
                if self.roverY != self.lastRoverYSent:
                    self.sendSensorUpdate( "roverY", self.roverY )
                    self.lastRoverYSent = self.roverY
                
                if self.headingDegrees != self.lastRoverHeadingDegreesSent:
                    self.sendSensorUpdate( "roverHeadingDegrees", self.headingDegrees )
                    self.lastRoverHeadingDegreesSent = self.headingDegrees
                    
                if self.ultrasonicRangeCM != self.lastUltrasonicRangeCMSent:
                    self.sendSensorUpdate( "ultrasonicRangeCM", self.ultrasonicRangeCM )
                    self.lastUltrasonicRangeCMSent = self.ultrasonicRangeCM
                    
                if self.artifactID != self.lastArtifactIDSent:
                    self.sendSensorUpdate( "artifactID", self.artifactID )
                    self.lastArtifactIDSent = self.artifactID
                    
                if self.artifactBearingDegrees != self.lastArtifactBearingDegreesSent:
                    self.sendSensorUpdate( "artifactBearingDegrees", self.artifactBearingDegrees )
                    self.lastArtifactBearingDegreesSent = self.artifactBearingDegrees

                
                if self.allCommandsComplete != self.lastAllCommandsCompleteSent:
                    
                    if self.allCommandsComplete:
                        self.sendSensorUpdate( "allCommandsComplete", 1 )
                    else:
                        self.sendSensorUpdate( "allCommandsComplete", 0 )
                        
                    self.lastAllCommandsCompleteSent = self.allCommandsComplete
                
                
                self.timeOfLastSensorUpdate = curTime
    
    #-----------------------------------------------------------------------------------------------
    def sendSensorUpdate(self, sensorName, value ):
        
        try:
            command = 'sensor-update "%s" %d' % ( sensorName, value)
            #print 'sending: %s' % command
            self.sendScratchCommand( command )
        except IOError as e:

            print "Exception when trying to send update"
            print e

            if e.errno == errno.EPIPE:
         
                if scratch_background.cycle_trace == 'running':
                        scratch_background.cycle_trace = 'disconnected'
            
    #-----------------------------------------------------------------------------------------------
    def sendScratchCommand( self, cmd ):
        n = len( cmd )
        length = scratch_background.int2byte( n )
        self.scratchSocket.send( length + cmd )

#---------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = scratch_background.DEFAULT_HOST

    BLUETOOTH_SERIAL_PORT = "/dev/rfcomm2"
    BLUETOOTH_SERIAL_BAUD_RATE = 9600

    bluetoothSerial = serial.Serial( BLUETOOTH_SERIAL_PORT, baudrate=BLUETOOTH_SERIAL_BAUD_RATE, timeout=0 )

    scratch_background.cycle_trace = 'start'

    while True:
        if (scratch_background.cycle_trace == 'disconnected'):
            print "Scratch disconnected"
            scratch_background.cleanupProcesses( ( listener, robotController ) )
            time.sleep(1)
            scratch_background.cycle_trace = 'start'

        if (scratch_background.cycle_trace == 'start'):
            # open the socket
            print 'Starting to connect...' ,
            scratchSocket = scratch_background.createSocket( host, scratch_background.PORT )
            print 'Connected!'
            scratchSocket.settimeout( scratch_background.SOCKET_TIMEOUT )
            
            commandQueue = multiprocessing.Queue()
            listener = scratch_background.ScratchListener( scratchSocket, commandQueue )
            robotController = RobotController( scratchSocket, commandQueue, bluetoothSerial )
            
            scratch_background.cycle_trace = 'running'
            print "Running...."
            listener.start()
            robotController.start()

        if (scratch_background.cycle_trace == 'quit'):
            scratch_background.cleanupProcesses( ( listener, robotController ) )
            sys.exit()

        # wait for a ctrl+c
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            scratch_background.cleanupProcesses( ( listener, robotController ) )
            sys.exit()
