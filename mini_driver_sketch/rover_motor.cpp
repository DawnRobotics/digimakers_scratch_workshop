#include <stdint.h>
#include "rover_motor.h"

//------------------------------------------------------------------------------
const unsigned long RoverMotor::MIN_TIME_DIFF_MS = 50;
const float RoverMotor::MIN_ABS_RPM = 5.0f;
const float RoverMotor::MAX_ABS_RPM = 100.0f;
const float RoverMotor::NUM_TICKS_PER_ROTATION = 1000.0f/3.0f;
const float RoverMotor::P_GAIN = 0.004f;
const float RoverMotor::I_GAIN = 0.0f;
const float RoverMotor::D_GAIN = 0.0005f;

//------------------------------------------------------------------------------
RoverMotor::RoverMotor( uint8_t dirPin, uint8_t pwmPin,
    uint8_t firstEncoderPin, uint8_t secondEncoderPin )
    : mDirPin( dirPin ),
    mPwmPin( pwmPin ),
    mEncoder( firstEncoderPin, secondEncoderPin ),
    mTargetRPM( 0.0 ),
    mLastMeasuredEncoderTicks( 0 ),
    mLastMeasuredRPM( 0.0 ),
    mCurDutyCycle( 0.0 ),
    mIntegralRPM( 0.0 ),
    mLastErrorRPM( 0.0 )
{
    pinMode( mDirPin, OUTPUT );
    pinMode( mPwmPin, OUTPUT );

    analogWrite( mPwmPin, 0 );
    digitalWrite( mDirPin, HIGH );
    
    mLastUpdateTimeMS = millis();
}

//------------------------------------------------------------------------------
void RoverMotor::update()
{   
    unsigned long curTimeMS = millis(); 
    unsigned long timeDiffMS = curTimeMS - mLastUpdateTimeMS;
    if ( timeDiffMS < MIN_TIME_DIFF_MS )
    {
        return;    // Not enough time has passed for another update
    }
    
    long encoderTicks = mEncoder.read();
    long encoderTicksDiff = encoderTicks - mLastMeasuredEncoderTicks;
    
    float encoderTicksPerMin = (float)(encoderTicksDiff*1000*60)/(float)timeDiffMS;
    mLastMeasuredRPM = encoderTicksPerMin/NUM_TICKS_PER_ROTATION;
    
    // Work out the change to make in the duty cycle
    float errorRPM = mTargetRPM - mLastMeasuredRPM;
    mIntegralRPM += errorRPM;
    float derivativeRPM = 1000.0f*(errorRPM - mLastErrorRPM)/(float)timeDiffMS;
    mLastErrorRPM = errorRPM;
    
    if ( 0.0 == mTargetRPM && abs( errorRPM ) < 2.5f )
    {
        mCurDutyCycle = 0.0;
    }
    else
    {
        mCurDutyCycle += errorRPM*P_GAIN + mIntegralRPM*I_GAIN + derivativeRPM*D_GAIN;
    }
    
    // Constrain the duty cycle
    mCurDutyCycle = constrain( mCurDutyCycle, -1.0, 1.0 );
    
    // Send the new control value to the motor
    if ( mCurDutyCycle >= 0.0 )
    {
        analogWrite( mPwmPin, mCurDutyCycle*255 );
        digitalWrite( mDirPin, LOW );
    }
    else
    {
        analogWrite( mPwmPin, -mCurDutyCycle*255 );
        digitalWrite( mDirPin, HIGH );
    }
    
    mLastMeasuredEncoderTicks = encoderTicks;
    mLastUpdateTimeMS = curTimeMS;
}

//------------------------------------------------------------------------------
void RoverMotor::setTargetRPM( float targetRPM )
{
    mTargetRPM = targetRPM;
    
    /*if ( mTargetRPM > 0.0 )
    {
        mTargetRPM = constrain( mTargetRPM, MIN_ABS_RPM, MAX_ABS_RPM );
    }
    else if ( mTargetRPM < 0.0 )
    {
        mTargetRPM = constrain( mTargetRPM, -MAX_ABS_RPM, -MIN_ABS_RPM );
    }*/
}


