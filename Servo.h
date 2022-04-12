/******************************************************************************
 * Servo.h
 * 
 * Servo control class. Controls a GPIO slice to provide a 20msec PWM control
 * signal to a servo. Default limits are used which work well for the servo
 * I am using. Means is provided to tweak the pulse timing limits.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 30Mar2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

#ifndef _SERVO_
#define _SERVO_

// Includes
//-----------------------------------------------------------------------------
#include "pico/stdlib.h"


// Defines
//-----------------------------------------------------------------------------
#define kServoPWMFreq           (50.0f)
#define kServoPWMPeriod         (1.0f / kServoPWMFreq)
#define kDefaultServoPWMMin     (450.0e-6f)
#define kDefaultServoPWMMax     (2650.0e-6f)


// Servo class
/// Controls a servo via a PWM channel mapped to a GPIO
//-----------------------------------------------------------------------------
class Servo {
    private:
        bool mbIsValid;
        uint muGPIO;            ///< 
        uint muSlice;
        uint muChan;
        uint muServoCountMin;
        uint muServoCountMax;
        bool mbLongestPulseIsMax;
        float mfCurrentPosn;

    public:
        /// Construct a Servo object managing the specified pin. User can
        /// specify polarity and timing parameters for PWM signal. The
        /// PWM signal operates at a 'standard' 20msec period (50Hz) and
        /// is active high.
        Servo(
            uint uGPIO, 
            float fInitialPosn=0.0f,
            bool bLongestPulseIsMaxPosition=true, 
            float fMinPulseSec=kDefaultServoPWMMin, 
            float fMaxPulseSec=kDefaultServoPWMMax
        );

        /// Destructor - switches off PWM and deallocates the GPIO
        ~Servo();

        /// Returns 'false' if setup is incorrect. Otherwise assume PWM channel
        /// is up and running.
        bool isValid(void) const;

        /// Set servo position in the range 0.0f to 1.0f
        void setPosition(float fPosition);
};

#endif // _SERVO_