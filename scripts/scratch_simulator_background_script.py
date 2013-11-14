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
import scratch_background

#---------------------------------------------------------------------------------------------------
class RobotSimulator( scratch_background.ScratchBase ):
    
    MIN_TIME_BETWEEN_SIM_STEPS = 0.05
    MIN_TIME_BETWEEN_SENSOR_UPDATES = 0.1
    
    MOVE_SPEED_PER_SECOND = 20.0        # In cm per second
    TURN_SPEED_PER_SECOND = 90.0/4.0    # In degrees per second
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, socket, commandQueue ):
        scratch_background.ScratchBase.__init__( self, socket )
        
        self.commandQueue = commandQueue
        self.timeOfLastSimulatorStep = time.time()
        self.timeOfLastSensorUpdate = time.time()
        
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
        
        self.roverX = 0
        self.roverY = 0
        self.headingDegrees = 90
        self.curCommand = None
        self.allCommandsComplete = False
        
        self.distanceMoved = 0
        self.distanceToMove = 0
        self.degreesToTurn = 0
        self.degreesTurned = 0
    
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        while not self.stopped():

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
                            self.distanceMoved = 0
                            
                        elif newCommand.startswith( "turn" ):
                            
                            self.curCommand = "turn"
                            self.degreesToTurn = self.parseNumber( newCommand[ len( "turn" ): ] )
                            self.degreesTurned = 0

            # Update the robot simulation
            curTime = time.time()
            if curTime - self.timeOfLastSimulatorStep >= self.MIN_TIME_BETWEEN_SIM_STEPS:
                
                timeDiff = curTime - self.timeOfLastSimulatorStep
                
                if self.curCommand == "move":
                    
                    headingX = math.sin( math.radians( self.headingDegrees ) )
                    headingY = math.cos( math.radians( self.headingDegrees ) )
                    
                    stepDistance = self.MOVE_SPEED_PER_SECOND*timeDiff
                    
                    if self.distanceToMove < 0:
                        stepDistance = -stepDistance
                        
                    self.roverX += stepDistance*headingX
                    self.roverY += stepDistance*headingY
                
                    self.distanceMoved += stepDistance    
                    
                    if abs( self.distanceMoved ) >= abs( self.distanceToMove ):
                        self.curCommand = None
                        self.allCommandsComplete = True
                        
                elif self.curCommand == "turn":
                    
                    stepAngle = self.TURN_SPEED_PER_SECOND*timeDiff
                    
                    if self.degreesToTurn < 0:
                        stepAngle = -stepAngle
                    
                    self.headingDegrees += stepAngle
                    self.degreesTurned += stepAngle
                    
                    # Normalise the heading
                    while self.headingDegrees < 0.0:
                        self.headingDegrees += 360.0
                    while self.headingDegrees >= 360.0:
                        self.headingDegrees -= 360.0
                    
                    if abs( self.degreesTurned ) >= abs( self.degreesToTurn ):
                        self.curCommand = None
                        self.allCommandsComplete = True
                
                self.timeOfLastSimulatorStep = curTime
            
            # Send current state back to scratch
            curTime = time.time()
            if curTime - self.timeOfLastSensorUpdate >= self.MIN_TIME_BETWEEN_SENSOR_UPDATES:
                
                self.sendSensorUpdate( "roverX", self.roverX )
                self.sendSensorUpdate( "roverY", self.roverY )
                self.sendSensorUpdate( "roverHeadingDegrees", self.headingDegrees )
                
                if self.allCommandsComplete:
                    self.sendSensorUpdate( "allCommandsComplete", 1 )
                else:
                    self.sendSensorUpdate( "allCommandsComplete", 0 )
                
                
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

    scratch_background.cycle_trace = 'start'

    while True:
        if (scratch_background.cycle_trace == 'disconnected'):
            print "Scratch disconnected"
            scratch_background.cleanupProcesses( ( listener, robotSimulator ) )
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
            robotSimulator = RobotSimulator( scratchSocket, commandQueue )
            
            scratch_background.cycle_trace = 'running'
            print "Running...."
            listener.start()
            robotSimulator.start()

        if (scratch_background.cycle_trace == 'quit'):
            scratch_background.cleanupProcesses( ( listener, robotSimulator ) )
            sys.exit()

        # wait for a ctrl+c
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            scratch_background.cleanupProcesses( ( listener, robotSimulator ) )
            sys.exit()
