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
import random

#---------------------------------------------------------------------------------------------------
def intersectLineSegments( o1, p1, o2, p2 ):
    
    intersectionPos = None
    distanceToIntersection = None
    
    oVec = ( o2[ 0 ] - o1[ 0 ], o2[ 1 ] - o1[ 1 ] )
    dir1 = ( p1[ 0 ] - o1[ 0 ], p1[ 1 ] - o1[ 1 ] )
    dir2 = ( p2[ 0 ] - o2[ 0 ], p2[ 1 ] - o2[ 1 ] )
    
    cross = dir1[ 0 ]*dir2[ 1 ] - dir1[ 1 ]*dir2[ 0 ];
    if abs(cross) < 1e-8:
        return intersectionPos, distanceToIntersection  # Parallel lines
    
    t = (oVec[ 0 ]*dir2[ 1 ] - oVec[ 1 ]*dir2[ 0 ])/cross
    
    intersectionPos = ( o1[ 0 ] + t*dir1[ 0 ], o1[ 1 ] + t*dir1[ 1 ] )
    dirLength1 = math.sqrt( dir1[ 0 ]**2 + dir1[ 1 ]**2 )
    distanceToIntersection = t*dirLength1
    
    if dir2[ 0 ] != 0:
    
        otherT = (intersectionPos[ 0 ] - o2[ 0 ])/dir2[ 0 ]
        if otherT < 0.0 or otherT > 1.0:
            
            intersectionPos = None
            distanceToIntersection = None

    return intersectionPos, distanceToIntersection
        
        

#---------------------------------------------------------------------------------------------------
class Obstacle:
    
    WIDTH_CM = 31.0
    HEIGHT_CM = 23.0
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, posX, posY, headingDegrees ):
    
        self.posX = posX
        self.posY = posY
        self.headingDegrees = headingDegrees
        
    #-----------------------------------------------------------------------------------------------
    def intersectUltrasonicRay( self, rayStartPos, rayEndPos ):
        
        oX = ( (self.WIDTH_CM/2.0)*math.sin( math.radians( self.headingDegrees ) ),
            (self.WIDTH_CM/2.0)*math.cos( math.radians( self.headingDegrees ) ) )
        oY = ( (self.HEIGHT_CM/2.0)*math.cos( math.radians( self.headingDegrees) ),
            (self.HEIGHT_CM/2.0)*math.sin( math.radians( self.headingDegrees ) ) )
        
        corners = [ 
            ( self.posX - oX[ 0 ] - oY[ 0 ], self.posY - oX[ 1 ] - oY[ 1 ] ),
            ( self.posX + oX[ 0 ] - oY[ 0 ], self.posY + oX[ 1 ] - oY[ 1 ] ),
            ( self.posX + oX[ 0 ] + oY[ 0 ], self.posY + oX[ 1 ] + oY[ 1 ] ),
            ( self.posX - oX[ 0 ] + oY[ 0 ], self.posY - oX[ 1 ] + oY[ 1 ] ) ]
        
        #for corner in corners:
         #   print "     ", corner
        
        sides = [
            ( corners[ 0 ], corners[ 1 ] ),
            ( corners[ 1 ], corners[ 2 ] ),
            ( corners[ 2 ], corners[ 3 ] ),
            ( corners[ 3 ], corners[ 0 ] ) ]
            
        minDistanceToIntersection = None
        for side in sides:
            
            intersectionPos, distanceToIntersection = intersectLineSegments(
                rayStartPos, rayEndPos, side[ 0 ], side[ 1 ] )
                
            if intersectionPos != None and distanceToIntersection > 0.0:
                
                #print intersectionPos, distanceToIntersection

                if minDistanceToIntersection == None \
                    or distanceToIntersection < minDistanceToIntersection:
                        
                    minDistanceToIntersection = distanceToIntersection
                    
        return minDistanceToIntersection

#---------------------------------------------------------------------------------------------------
class Artifact:
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, artifactID, posX, posY ):
        
        self.artifactID = artifactID
        self.posX = posX
        self.posY = posY

#---------------------------------------------------------------------------------------------------
class RobotSimulator( scratch_background.ScratchBase ):
    
    MIN_TIME_BETWEEN_SIM_STEPS = 0.05
    MIN_TIME_BETWEEN_SENSOR_UPDATES = 0.1
    
    MOVE_SPEED_PER_SECOND = 20.0        # In cm per second
    TURN_SPEED_PER_SECOND = 90.0/4.0    # In degrees per second
    MAX_ULTRASONIC_RANGE_CM = 400
    MAX_NUM_BEEPS = 9
    TIME_FOR_BEEP = 0.4
    BEEP_ON_TIME = 0.3
    SIMULATED_SCAN_TIME = 5.0
    MAX_ARTIFACT_DISTANCE = 50.0
    
    ROVER_LENGTH = 25.0
    
    OBSTACLES = [
        Obstacle( 0, -50.0, 90.0 ),
        Obstacle( Obstacle.WIDTH_CM/2.0 + Obstacle.HEIGHT_CM/2.0, -50.0 + Obstacle.WIDTH_CM/2.0 + Obstacle.HEIGHT_CM/2.0, 0.0 ),
        Obstacle( Obstacle.WIDTH_CM/2.0 + Obstacle.HEIGHT_CM/2.0, -50.0 + 3.0*Obstacle.WIDTH_CM/2.0 + Obstacle.HEIGHT_CM/2.0, 0.0 ),
        Obstacle( -100.0, -50.0 + Obstacle.WIDTH_CM/4.0, 0.0 ),
        Obstacle( -74.0, -16.0, 90.0 ),
        Obstacle( -70.0, 10.0, 90.0 )
    ]
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, socket, commandQueue ):
        scratch_background.ScratchBase.__init__( self, socket )
        
        self.commandQueue = commandQueue
        self.timeOfLastSimulatorStep = time.time()
        self.timeOfLastSensorUpdate = time.time()
        self.ultrasonicRangeCM = self.MAX_ULTRASONIC_RANGE_CM
        self.artifactID = -1
        self.artifactBearingDegrees = 0.0
        
        for i, obstacle in enumerate( self.OBSTACLES ):
                    
            print "Obstacle_{0}_X".format( i + 1 ), obstacle.posX
            print "Obstacle_{0}_Y".format( i + 1 ), obstacle.posY
            print "Obstacle_{0}_HeadingDegrees".format( i + 1 ), obstacle.headingDegrees
        
        
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
        self.buzzerOn = False
        self.curCommand = None
        self.allCommandsComplete = False
        
        self.lastRoverXSent = -10000
        self.lastRoverYSent = -10000
        self.lastRoverHeadingDegreesSent = -10000
        self.lastBuzzerOnSent = -10000
        self.lastUltrasonicRangeCMSent = -10000
        self.lastArtifactIDSent = -10000
        self.lastArtifactBearingDegreesSent = -10000
        self.lastAllCommandsCompleteSent = -10000
        self.lastArtifactSent = None
        
        self.distanceMoved = 0
        self.distanceToMove = 0
        self.degreesToTurn = 0
        self.degreesTurned = 0
        
        if random.random() < 0.5:
            
            self.artifact = Artifact( 3, 27, 28 )
            
        else:
            
            self.artifact = Artifact( 5, -70, 29 )
    
    #---------------------------------------------------------------------------------------------------
    def readDistanceFromSonarCM( self ):
        
        distanceCM = self.MAX_ULTRASONIC_RANGE_CM
        
        headingX = math.sin( math.radians( self.headingDegrees ) )
        headingY = math.cos( math.radians( self.headingDegrees ) )
        
        halfRoverLength = self.ROVER_LENGTH/2.0
        rayStartX = self.roverX + halfRoverLength*headingX
        rayStartY = self.roverY + halfRoverLength*headingY
        rayEndX = rayStartX + self.MAX_ULTRASONIC_RANGE_CM*headingX
        rayEndY = rayStartY + self.MAX_ULTRASONIC_RANGE_CM*headingY
        
        # Test against each obstacle in turn
        for i, obstacle in enumerate( self.OBSTACLES ):
            
            #print "obstacle", i
            
            distanceToIntersectionCM = obstacle.intersectUltrasonicRay( ( rayStartX, rayStartY ), ( rayEndX, rayEndY ) )
            
            if distanceToIntersectionCM != None:
                if distanceToIntersectionCM < distanceCM:
                    
                    distanceCM = distanceToIntersectionCM
        
        return distanceCM
    
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        while not self.stopped():

            loopStartTime = time.time()

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
                            
                        elif newCommand.startswith( "beep" ):
                        
                            self.curCommand = "beep"
                            self.numBeepsToMake = self.parseNumber( newCommand[ len( "beep" ): ] )
                            if self.numBeepsToMake > self.MAX_NUM_BEEPS:
                                self.numBeepsToMake = self.MAX_NUM_BEEPS
                            self.numBeepsMade = 0
                            self.waitingForBeepToComplete = False
                            
                        elif newCommand == "detectartifact":
                            
                            self.curCommand = "detectartifact"
                            self.scanStartTime = time.time()

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
                
                elif self.curCommand == "beep":
                
                    self.buzzerOn = False
                
                    if self.waitingForBeepToComplete \
                        and curTime - self.timeBeepStarted >= self.TIME_FOR_BEEP:
                        
                        self.numBeepsMade += 1
                        self.waitingForBeepToComplete = False
                        
                    if self.numBeepsMade >= self.numBeepsToMake:
                        
                        self.curCommand = None
                        self.allCommandsComplete = True
                        
                    else:
                        
                        if not self.waitingForBeepToComplete:
                            # Start beep
                            self.buzzerOn = True
                            self.waitingForBeepToComplete = True
                            self.timeBeepStarted = time.time()
                            
                        elif curTime - self.timeBeepStarted <= self.BEEP_ON_TIME:
                            
                            self.buzzerOn = True
                
                elif self.curCommand == "detectartifact":
                    
                    if curTime - self.scanStartTime >= self.SIMULATED_SCAN_TIME:
                        
                        # Look to see if the artifact is in view
                        artifactVecX = self.artifact.posX - self.roverX
                        artifactVecY = self.artifact.posY - self.roverY
                        
                        headingX = math.sin( math.radians( self.headingDegrees ) )
                        headingY = math.cos( math.radians( self.headingDegrees ) )
                        
                        distanceToArtifact = math.sqrt( artifactVecX**2 + artifactVecY**2 )
                        if distanceToArtifact <= self.MAX_ARTIFACT_DISTANCE:
                            
                            print "Artifact close"
                            
                            self.artifactID = self.artifact.artifactID
                            self.artifactBearingDegrees = 0.0
                            
                            if distanceToArtifact > 0.0:
                                artifactDirX = artifactVecX/distanceToArtifact
                                artifactDirY = artifactVecY/distanceToArtifact
                                
                                cosOfAngle = artifactDirX*headingX + artifactDirY*headingY
                                self.artifactBearingDegrees = math.degrees( math.acos( cosOfAngle ) )
                                
                                cosOfSideAngle = artifactDirX*-headingY + artifactDirY*headingX
                                if cosOfSideAngle >= 0.0:
                                    self.artifactBearingDegrees = -self.artifactBearingDegrees
                            
                        else:
                            
                            self.artifactID = -1
                            self.artifactBearingDegrees = 0.0
                            
                        # Finished detection
                        self.curCommand = None
                        self.allCommandsComplete = True
                
                self.timeOfLastSimulatorStep = curTime
            
            # Simulate the ultrasonic sensor
            self.ultrasonicRangeCM = self.readDistanceFromSonarCM()
            
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
                
                if self.buzzerOn != self.lastBuzzerOnSent:
                    self.sendSensorUpdate( "buzzerOn", self.buzzerOn )
                    self.lastBuzzerOnSent = self.buzzerOn
                
                if self.ultrasonicRangeCM != self.lastUltrasonicRangeCMSent:
                    self.sendSensorUpdate( "ultrasonicRangeCM", self.ultrasonicRangeCM )
                    self.lastUltrasonicRangeCMSent = self.ultrasonicRangeCM
                    
                if self.artifactID != self.lastArtifactIDSent:
                    self.sendSensorUpdate( "artifactID", self.artifactID )
                    self.lastArtifactIDSent = self.artifactID
                    
                if self.artifactBearingDegrees != self.lastArtifactBearingDegreesSent:
                    self.sendSensorUpdate( "artifactBearingDegrees", self.artifactBearingDegrees )
                    self.lastArtifactBearingDegreesSent = self.artifactBearingDegrees

                if self.artifact != self.lastArtifactSent:
                    
                    self.sendSensorUpdate( "artifactX", self.artifact.posX )
                    self.sendSensorUpdate( "artifactY", self.artifact.posY )
                    self.lastArtifactSent = self.artifact
                
                if self.allCommandsComplete != self.lastAllCommandsCompleteSent:
                    
                    if self.allCommandsComplete:
                        self.sendSensorUpdate( "allCommandsComplete", 1 )
                    else:
                        self.sendSensorUpdate( "allCommandsComplete", 0 )
                        
                    self.lastAllCommandsCompleteSent = self.allCommandsComplete
                
                #for i, obstacle in enumerate( self.OBSTACLES ):
                    
                    #self.sendSensorUpdate( "Obstacle_{0}_X".format( i + 1 ), obstacle.posX )
                    #self.sendSensorUpdate( "Obstacle_{0}_Y".format( i + 1 ), obstacle.posY )
                    #self.sendSensorUpdate( "Obstacle_{0}_HeadingDegrees".format( i + 1 ), obstacle.headingDegrees )
                
                self.timeOfLastSensorUpdate = curTime
                
            loopEndTime = time.time()
            minIdealLoopTime = min( self.MIN_TIME_BETWEEN_SIM_STEPS, self.MIN_TIME_BETWEEN_SENSOR_UPDATES )
            elapsedTime = loopEndTime - loopStartTime
            remainingTime = minIdealLoopTime - elapsedTime
            
            if remainingTime > 0.0:
                #print remainingTime, "remaining"
                time.sleep( remainingTime )
    
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
