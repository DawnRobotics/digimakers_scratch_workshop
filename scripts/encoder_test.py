#! /usr/bin/env python

# Encoder test

import RPIO
import serial
import time

#---------------------------------------------------------------------------------------------------
LEFT_ENCODER_PIN_1 = 27
LEFT_ENCODER_PIN_2 = 22
RIGHT_ENCODER_PIN_1 = 4
RIGHT_ENCODER_PIN_2 = 17

#---------------------------------------------------------------------------------------------------
class Encoder:

    PIN_STATE_TO_BINARY_NUMBER = {
        RPIO.LOW : {
            RPIO.LOW : 0,
            RPIO.HIGH : 1
        },
        RPIO.HIGH : {
            RPIO.LOW : 2,
            RPIO.HIGH : 3
        }
    }

    DUAL_INTERRUPT_TRANSITION_TABLE = [
        [ 0, -1, 1, 2 ],
        [ 1, 0, 2, -1 ],
        [ -1, 2, 0, 1 ],
        [ 2, 1, -1, 0 ]
    ]
    
    SINGLE_INTERRUPT_TRANSITION_TABLE = [
        [ 0, -1, 1, 2 ],
        [ 1, 0, 2, -1 ],
        [ -1, 2, 0, 1 ],
        [ 2, 1, -1, 0 ]
    ]

    #-----------------------------------------------------------------------------------------------
    def __init__( self, firstPin, secondPin ):

        self.firstPin = firstPin
        self.secondPin = secondPin
        RPIO.setup( self.firstPin, RPIO.IN )
        RPIO.setup( self.secondPin, RPIO.IN )
        self.firstPinState = RPIO.input( self.firstPin )
        self.secondPinState = RPIO.input( self.secondPin )
        
        RPIO.add_interrupt_callback( self.firstPin, self.encoderCallback )
        RPIO.add_interrupt_callback( self.secondPin, self.encoderCallback )
        
        self.tickCount = 0
        self.edgeMissed = False
        
    #-----------------------------------------------------------------------------------------------
    def encoderCallback( self, pin, value ):
        
        if pin == self.firstPin:
            newFirstPinState = value
            newSecondPinState = self.secondPinState
        elif pin == self.secondPin:
            newFirstPinState = self.firstPinState
            newSecondPinState = value
        else:
            print "Invalid pin"
            return
            
        #newFirstPinState = RPIO.input( self.firstPin )
        #newSecondPinState = RPIO.input( self.secondPin )
        
        oldNumber = self.PIN_STATE_TO_BINARY_NUMBER[ self.firstPinState ][ self.secondPinState ]
        newNumber = self.PIN_STATE_TO_BINARY_NUMBER[ newFirstPinState ][ newSecondPinState ]
        
        tickChange = self.DUAL_INTERRUPT_TRANSITION_TABLE[ oldNumber ][ newNumber ] 
        if abs( tickChange ) > 1:
            print "Warning: missed edge..."
            self.edgeMissed = True
        
        self.tickCount += tickChange
        self.firstPinState = newFirstPinState
        self.secondPinState = newSecondPinState
        
        print self.tickCount
    
    #-----------------------------------------------------------------------------------------------
    def read( self ):
        
        return self.tickCount
        
    #-----------------------------------------------------------------------------------------------
    def write( self, newTickCount ):
        
        self.tickCount = newTickCount

#---------------------------------------------------------------------------------------------------
RPIO.setmode( RPIO.BCM )
RPIO.wait_for_interrupts( threaded=True )
leftEncoder = Encoder( LEFT_ENCODER_PIN_1, LEFT_ENCODER_PIN_2 )

while True:
    pass
    #print leftEncoder.read()