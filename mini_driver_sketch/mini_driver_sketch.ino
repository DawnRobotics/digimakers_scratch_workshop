// Copyright (c) 2013 Dawn Robotics Ltd - Alan Broun <abroun@dawnrobotics.co.uk>

#include <stdint.h>
#include "rover_motor.h"


const int LEFT_DIR_PIN = 7;
const int LEFT_PWM_PIN = 9;
const int LEFT_ENCODER_FIRST_PIN = 3;
const int LEFT_ENCODER_SECOND_PIN = 5;

const int RIGHT_DIR_PIN = 8;
const int RIGHT_PWM_PIN = 10;
const int RIGHT_ENCODER_FIRST_PIN = 2;
const int RIGHT_ENCODER_SECOND_PIN = 4;

const float ABS_MOVE_RPM = 40.0f;
const float ABS_TURN_RPM = 30.0f;

RoverMotor gLeftMotor( LEFT_DIR_PIN, LEFT_PWM_PIN,
    LEFT_ENCODER_FIRST_PIN, LEFT_ENCODER_SECOND_PIN );
RoverMotor gRightMotor( RIGHT_DIR_PIN, RIGHT_PWM_PIN,
    RIGHT_ENCODER_FIRST_PIN, RIGHT_ENCODER_SECOND_PIN );

//------------------------------------------------------------------------------
void setup()
{    
    Serial.begin( 9600 );
}

//------------------------------------------------------------------------------
void loop()
{
    gLeftMotor.update();
    gRightMotor.update();
    
    while ( Serial.available() )
    {
        char command = Serial.read();
        switch ( command )
        {
        case 'f':
            {
                gLeftMotor.setTargetRPM( ABS_MOVE_RPM );
                gRightMotor.setTargetRPM( ABS_MOVE_RPM );
                break;
            }
        case 'b':
            {
                gLeftMotor.setTargetRPM( -ABS_MOVE_RPM );
                gRightMotor.setTargetRPM( -ABS_MOVE_RPM );
                break;
            }
        case 'l':
            {
                gLeftMotor.setTargetRPM( -ABS_TURN_RPM );
                gRightMotor.setTargetRPM( ABS_TURN_RPM );
                break;
            }
        case 'r':
            {
                gLeftMotor.setTargetRPM( ABS_TURN_RPM );
                gRightMotor.setTargetRPM( -ABS_TURN_RPM );
                break;
            }
        case 's':
        default:
            {
                gLeftMotor.setTargetRPM( 0.0 );
                gRightMotor.setTargetRPM( 0.0 );
                break;
            }
        }
    }
    
    // Output debug info here
    Serial.print( gLeftMotor.getLastMeasuredEncoderTicks() );
    Serial.print( " " );
    Serial.println( gRightMotor.getLastMeasuredEncoderTicks() );
}


