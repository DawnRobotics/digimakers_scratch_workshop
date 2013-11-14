#! /usr/bin/env python

# Control software for the Raspberry Pi Rover 5 Robot

import RPIO
import RPIO.PWM
import serial
import time

#---------------------------------------------------------------------------------------------------
# Constants
BLUETOOTH_BAUD_RATE = 9600

LEFT_MOTOR_PWM_PIN = 23
LEFT_MOTOR_DIR_PIN = 18
RIGHT_MOTOR_PWM_PIN = 2
RIGHT_MOTOR_DIR_PIN = 3
SONAR_PIN = 7

PWM_FREQUENCY = 50    # Hz

PWM_DMA_CHANNEL = 0
PWM_SUBCYLCLE_TIME_US = 1000/PWM_FREQUENCY * 1000
PWM_PULSE_INCREMENT_US = 10

#UPDATE_FREQUENCY = 50   # Gives the rate in Hz we should run the update loop at
#IDEAL_TIME_DIFF_BETWEEN_UPDATES = 1.0/UPDATE_FREQUENCY

FORWARD_DIR = RPIO.LOW
BACKWARD_DIR = RPIO.HIGH

BACKUP_DISTANCE = 0.1
BACKUP_TIME = 0.5

PAN_PWM_PIN = 24
TILT_PWM_PIN = 25

ABSOLUTE_MIN_PULSE_WIDTH_US = 500
ABSOLUTE_MAX_PULSE_WIDTH_US = 2500

EXPLORE_PAN_START_ANGLE = 45.0
EXPLORE_PAN_END_ANGLE = 135.0
EXPLORE_TILT_START_ANGLE = 130.0
EXPLORE_TILT_END_ANGLE = 50.0

EXPLORE_ABS_PAN_STEP = 5
EXPLORE_ABS_TILT_STEP = 5

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IMAGE_JPEG_QUALITY = 85

#-------------------------------------------------------------------------------
class ServoPWM:
    
    #---------------------------------------------------------------------------
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
    
    #---------------------------------------------------------------------------
    def setCommand( self, command ):
        
        # Work out whether the command is an angle, or a pulse width
        if command >= ABSOLUTE_MIN_PULSE_WIDTH_US:
            self.setPulseWidth( command )
        else:
            self.setAngle( command )
    
    #---------------------------------------------------------------------------
    def setPulseWidth( self, pulseWidth ):
        
        # Constrain the pulse width
        if pulseWidth < ABSOLUTE_MIN_PULSE_WIDTH_US:
            pulseWidth = ABSOLUTE_MIN_PULSE_WIDTH_US
        if pulseWidth > ABSOLUTE_MAX_PULSE_WIDTH_US:
            pulseWidth = ABSOLUTE_MAX_PULSE_WIDTH_US
        
        # Ensure that the pulse width is an integer multiple of the smallest 
        # possible pulse increment
        pulseIncrementUS = RPIO.PWM.get_pulse_incr_us()
        numPulsesNeeded = int( pulseWidth/pulseIncrementUS )
        pulseWidth = numPulsesNeeded * pulseIncrementUS
    
        if pulseWidth != self.lastPulseWidthSet:
        
            RPIO.PWM.add_channel_pulse( PWM_DMA_CHANNEL, self.pwmPin, 0, numPulsesNeeded )
            self.lastPulseWidthSet = pulseWidth
    
    #---------------------------------------------------------------------------
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
class RobotState:
    """Collects all information about the robot into one place"""
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        
        self.driveDirection = "DD_Stopped"
        self.turnDirection = "TD_None"
        self.backingUp = False
        self.backupStartTime = 0
        
#---------------------------------------------------------------------------------------------------
def processBluetoothCommands( bluetoothSerial, robotState ):
    
    allCommandsProcessed = False
    while not allCommandsProcessed:
        
        commandBytes = bluetoothSerial.read( 1 )
        
        if len( commandBytes ) == 0:
            
            allCommandsProcessed = True
           
        else:
            
            command = commandBytes[ 0 ]
           
            if command == ord( 'f' ):
                
                robotState.driveDirection = "DD_Forward"
                
            elif command == ord( 'b' ):
                
                robotState.driveDirection = "DD_Backward"
                
            elif command == ord( 'l' ):
                
                robotState.turnDirection = "TD_Left"
                
            elif command == ord( 'r' ):
                
                robotState.turnDirection = "TD_Right"
            
            elif command == ord( 'z' ) or command == ord( 'y' ):    # Stop forward or backward
            
                robotState.driveDirection = "DD_Stopped"
            
            elif command == ord( 'v' ):     # Stop turn

                robotState.turnDirection = "TD_None"
            
            elif command == ord( 's' ):
                
                robotState.driveDirection = "DD_Stopped"
                robotState.turnDirection = "TD_None"

#---------------------------------------------------------------------------------------------------
def applyRobotStateToHardware( leftMotorPWM, rightMotorPWM, robotState ):
    
    # Work out the commands to send to the motors
    if robotState.driveDirection == "DD_Forward":
        
        leftDutyCycle = 100.0
        rightDutyCycle = 100.0
        leftDir = FORWARD_DIR
        rightDir = FORWARD_DIR
        
        if robotState.turnDirection == "TD_Left":
            leftDutyCycle = 100.0
            rightDutyCycle = 25.0
        elif robotState.turnDirection == "TD_Right":
            leftDutyCycle = 25.0
            rightDutyCycle = 100.0
            
    elif robotState.driveDirection == "DD_Backward":
        
        leftDutyCycle = 75.0
        rightDutyCycle = 75.0
        leftDir = BACKWARD_DIR
        rightDir = BACKWARD_DIR
        
        if robotState.turnDirection == "TD_Left":
            leftDutyCycle = 25.0
            rightDutyCycle = 100.0
        elif robotState.turnDirection == "TD_Right":
            leftDutyCycle = 100.0
            rightDutyCycle = 25.0
            
    else:   # DD_Stopped
    
        leftDutyCycle = 0.0
        rightDutyCycle = 0.0
        leftDir = FORWARD_DIR
        rightDir = FORWARD_DIR
        
        if robotState.turnDirection == "TD_Left":
            leftDutyCycle = 100.0
            rightDutyCycle = 100.0
            leftDir = FORWARD_DIR
            rightDir = BACKWARD_DIR
        elif robotState.turnDirection == "TD_Right":
            leftDutyCycle = 100.0
            rightDutyCycle = 100.0
            leftDir = BACKWARD_DIR
            rightDir = FORWARD_DIR
            
    # Send the motor commands
    leftDutyCycle = 100.0
    rightDutyCycle = 50.0
    leftDir = FORWARD_DIR
    rightDir = BACKWARD_DIR
    
    print( "Setting duty cycles of {0} and {1}".format( leftDutyCycle, rightDutyCycle ) )
    leftMotorPWM.ChangeDutyCycle( leftDutyCycle )
    GPIO.output( LEFT_MOTOR_DIR_PIN, leftDir )
    #rightMotorPWM.ChangeDutyCycle( rightDutyCycle )
    #GPIO.output( RIGHT_MOTOR_DIR_PIN, rightDir )

#---------------------------------------------------------------------------------------------------
def readDistanceFromSonar():
    
    distance = 0.0
    
    # Send the start signal
    GPIO.setup( SONAR_PIN, GPIO.OUT )
    GPIO.output( SONAR_PIN, GPIO.LOW )
    #time.sleep( 0.00002 )
    
    GPIO.output( SONAR_PIN, GPIO.HIGH )
    time.sleep( 0.00001 )
    GPIO.output( SONAR_PIN, GPIO.LOW )
    
    # Prepare to receive the response
    GPIO.setup( SONAR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN )
    
    # TODO: It would be nicer to use interrupts here, but I can't get them to work just yet...
    
    #GPIO.wait_for_edge( SONAR_PIN, GPIO.RISING ) 
    #startTime = time.time()
    #GPIO.wait_for_edge( SONAR_PIN, GPIO.FALLING )
    #endTime = time.time()
    
    startTime = time.time()
    while GPIO.input( SONAR_PIN ) == 0:
        startTime = time.time()

    while GPIO.input( SONAR_PIN ) == 1:
        endTime = time.time()
    
    measuredTimeUS = (endTime - startTime)*1000000.0
    distanceCM = measuredTimeUS / 58.0    # Using formula on http://www.seeedstudio.com/wiki/Ultra_Sonic_range_measurement_module
    distance = distanceCM / 100.0
    
    
    return distance

#-------------------------------------------------------------------------------
def finishedPanningForward( curPanAngle ):
        
    if EXPLORE_PAN_START_ANGLE < EXPLORE_PAN_END_ANGLE:
        return curPanAngle >= EXPLORE_PAN_END_ANGLE
    else:
        return curPanAngle <= EXPLORE_PAN_END_ANGLE
        
#-------------------------------------------------------------------------------
def finishedPanningBackward( curPanAngle ):
        
    if EXPLORE_PAN_START_ANGLE < EXPLORE_PAN_END_ANGLE:
        return curPanAngle <= EXPLORE_PAN_START_ANGLE
    else:
        return curPanAngle >= EXPLORE_PAN_START_ANGLE
        
#-------------------------------------------------------------------------------
def finishedTiltingForward( curTiltAngle ):
        
    if EXPLORE_TILT_START_ANGLE < EXPLORE_TILT_END_ANGLE:
        return curTiltAngle >= EXPLORE_TILT_END_ANGLE
    else:
        return curTiltAngle <= EXPLORE_TILT_END_ANGLE
        
#-------------------------------------------------------------------------------
def finishedTiltingBackward( curTiltAngle ):
        
    if EXPLORE_TILT_START_ANGLE < EXPLORE_TILT_END_ANGLE:
        return curTiltAngle <= EXPLORE_TILT_START_ANGLE
    else:
        return curTiltAngle >= EXPLORE_TILT_START_ANGLE
    
#---------------------------------------------------------------------------------------------------
# Main program
try:
    robotState = RobotState()

    # Create ServoPWM instances to control the servos
    panServoPWM = ServoPWM( PAN_PWM_PIN, 
        minAnglePulseWidthPair=( 10.0, 2000 ), 
        midAnglePulseWidthPair=( 90.0, 1250 ), 
        maxAnglePulseWidthPair=( 170.0, 500.0 ) )
    tiltServoPWM = ServoPWM( TILT_PWM_PIN, 
        minAnglePulseWidthPair=( 45.0, 1800 ), 
        midAnglePulseWidthPair=( 90.0, 1400 ), 
        maxAnglePulseWidthPair=( 180.0, 500.0 ) )
    
    # Setup serial communication to talk over bluetooth
    bluetoothSerial = serial.Serial( "/dev/ttyAMA0", BLUETOOTH_BAUD_RATE, timeout=0.0 )

    # Setup the GPIO pins to talk to the robot hardware
    # Setup RPIO, and prepare for PWM signals
    RPIO.setmode( RPIO.BCM )

    RPIO.PWM.setup( pulse_incr_us=PWM_PULSE_INCREMENT_US )
    RPIO.PWM.init_channel( PWM_DMA_CHANNEL, PWM_SUBCYLCLE_TIME_US )
    
    #RPIO.setup( LEFT_MOTOR_PWM_PIN, RPIO.OUT )
    RPIO.setup( LEFT_MOTOR_DIR_PIN, RPIO.OUT )
    #RPIO.setup( RIGHT_MOTOR_PWM_PIN, RPIO.OUT )
    RPIO.setup( RIGHT_MOTOR_DIR_PIN, RPIO.OUT )

    # Clear existing PWM
    #RPIO.PWM.clear_channel_gpio( PWM_DMA_CHANNEL, LEFT_MOTOR_PWM_PIN )
    #RPIO.PWM.clear_channel_gpio( PWM_DMA_CHANNEL, RIGHT_MOTOR_PWM_PIN )
    

    # Set to 100% duty cycle
    pulseIncrementUS = RPIO.PWM.get_pulse_incr_us()
    pulseWidth = 0 #(PWM_SUBCYLCLE_TIME_US) / pulseIncrementUS
    
    RPIO.PWM.add_channel_pulse( PWM_DMA_CHANNEL, LEFT_MOTOR_PWM_PIN, 0, pulseWidth )
    RPIO.PWM.add_channel_pulse( PWM_DMA_CHANNEL, RIGHT_MOTOR_PWM_PIN, 0, pulseWidth )
    
    # Perform exploration
    curPanAngle = EXPLORE_PAN_START_ANGLE
    curTiltAngle = EXPLORE_TILT_START_ANGLE
    panServoPWM.setAngle( curPanAngle )
    tiltServoPWM.setAngle( curTiltAngle )
    time.sleep( 0.1 )
    
    # Work out how to take pan and tilt steps
    if EXPLORE_PAN_START_ANGLE < EXPLORE_PAN_END_ANGLE:
        panForwardStep = EXPLORE_ABS_PAN_STEP
    else:
        panForwardStep = -EXPLORE_ABS_PAN_STEP
        
    if EXPLORE_TILT_START_ANGLE < EXPLORE_TILT_END_ANGLE:
        tiltForwardStep = EXPLORE_ABS_TILT_STEP
    else:
        tiltForwardStep = -EXPLORE_ABS_TILT_STEP
    
    explorationDirection = "PanningForward"
    finishedExploration = False
    
    while not finishedExploration:
        
        if explorationDirection == "PanningForward":
            
            if finishedPanningForward( curPanAngle ):
                
                # Reached the end of a row
                if finishedTiltingForward( curTiltAngle ):
                    
                    # Also reached the end of all rows
                    finishedExploration = True
                    
                else:
                    
                    curTiltAngle += tiltForwardStep
                    explorationDirection = "PanningBackward";
                
            else:
                
                curPanAngle += panForwardStep
        
        elif explorationDirection == "PanningBackward":
            
            if finishedPanningBackward( curPanAngle ):
                
                # Reached the beginning of a row
                if finishedTiltingForward( curTiltAngle ):
                    
                    # Also reached the end of all rows
                    finishedExploration = True
                    
                else:
                    
                    curTiltAngle += tiltForwardStep
                    explorationDirection = "PanningForward";
                
            else:
                
                curPanAngle -= panForwardStep

        panServoPWM.setAngle( curPanAngle )
        tiltServoPWM.setAngle( curTiltAngle )
                
        time.sleep( 0.01 )

        #img = picam.takePhotoWithDetails( IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_JPEG_QUALITY )
        #img.save( "p{0}_t{1}.jpg".format( curPanAngle, curTiltAngle ) )
        
        #frame1 = picam.takeRGBPhotoWithDetails( IMAGE_WIDTH, IMAGE_HEIGHT )
        
    # Return to the centre
    panServoPWM.setAngle( panServoPWM.midAnglePulseWidthPair[ 0 ] )
    tiltServoPWM.setAngle( tiltServoPWM.midAnglePulseWidthPair[ 0 ] )
    
    while True:
        
        pass
    
    # Enter into an update loop to run the robot
    #lastUpdateTime = 0
    #while True:

        #curTime = time.time()
        #timeDiff = curTime - lastUpdateTime
        #if timeDiff >= IDEAL_TIME_DIFF_BETWEEN_UPDATES:
            
            ## Update the robot
            
            ## Process any commands coming in via bluetooth
            #processBluetoothCommands( bluetoothSerial, robotState )
            
            ## Read the sensors
            #sonarDistance = 100.0 #readDistanceFromSonar()
            #if sonarDistance <= BACKUP_DISTANCE:
                
                #robotState.backingUp = True
                #robotState.backupStartTime = curTime
            
            ## Update the backing up behaviour
            #if robotState.backingUp:
                
                #robotState.driveDirection = "DD_Backward"
                #robotState.turnDirection = "TD_None"
                
                #if curTime - robotState.backupStartTime > BACKUP_TIME:
                    #robotState.backingUp = False
                    #robotState.driveDirection = "DD_Stopped"
            
            ## Send the state to the hardware
            #applyRobotStateToHardware( leftMotorPWM, rightMotorPWM, robotState )
            
            #lastUpdateTime = curTime
            
        #else:
            
            ## Not enough time has passed since the last update so sleep to pass the time
            #time.sleep( IDEAL_TIME_DIFF_BETWEEN_UPDATES - timeDiff )

except Exception as e:
    print "Got exception"
    print e
finally:
    RPIO.PWM.clear_channel_gpio( PWM_DMA_CHANNEL, LEFT_MOTOR_PWM_PIN )
    RPIO.PWM.clear_channel_gpio( PWM_DMA_CHANNEL, RIGHT_MOTOR_PWM_PIN )
    RPIO.PWM.clear_channel_gpio( PWM_DMA_CHANNEL, PAN_PWM_PIN )
    RPIO.PWM.clear_channel_gpio( PWM_DMA_CHANNEL, TILT_PWM_PIN )
    
    RPIO.PWM.cleanup()
    RPIO.cleanup()

