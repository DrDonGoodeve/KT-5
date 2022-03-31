/******************************************************************************
 * Servo.cpp
 * 
 * Servo control class. Controls a GPIO slice to provide a 20msec PWM control
 * signal to a servo. Default limits are used which work well for the servo
 * I am using. Means is provided to tweak the pulse timing limits.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 30Mar2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

// Includes
//-----------------------------------------------------------------------------
#include "Servo.h"
#include <math.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"


// Defines
//-----------------------------------------------------------------------------
#define kPicoClock      (125.0e6f)
#define kPWMCountMax    (0x10000)
#define kPWMFullFreq    (kPicoClock / (float)kPWMCountMax)
#define kClockDivider   ((uint)ceilf(kPWMFullFreq / kServoPWMFreq))
#define kDividedFreq    (kPWMFullFreq / (float)kClockDivider)
#define kDividedClk     (kPicoClock / (float)kClockDivider)
#define kDividedClkPer  (1.0f / kDividedClk)
#define kUncorrectedPer (1.0f / (float)kDividedFreq)
#define kExcessTime     (kUncorrectedPer - kServoPWMPeriod)
#define kExcessClocks   ((uint)roundf(kExcessTime / kDividedClkPer))
#define kCountMax       (kPWMCountMax - kExcessClocks)
#define kExactPeriod    ((float)kCountMax * kDividedClkPer)
#define kExactFreq      (1.0f / kExactPeriod)


// Servo:: implementation
//-----------------------------------------------------------------------------
Servo::Servo(
    uint uGPIO, float fInitialPosition, bool bLongestPulseIsMaxPosition, 
    float fMinPulseSec, float fMaxPulseSec) :
    mbIsValid(false), mbLongestPulseIsMax(bLongestPulseIsMaxPosition),
    muGPIO(uGPIO), muSlice(pwm_gpio_to_slice_num(uGPIO)), muChan(pwm_gpio_to_channel(uGPIO)) {

    // Sanity checks
    if (fMinPulseSec >= fMaxPulseSec) {
        return;
    }
    mbIsValid = true;

    // Determine minimum and maximum counts
    muServoCountMin = ((uint)roundf((fMinPulseSec / kExactPeriod) * (float)kCountMax));
    muServoCountMax = ((uint)roundf((fMaxPulseSec / kExactPeriod) * (float)kCountMax));

    // Determine PWM slice and channel for GPIO
    muSlice = pwm_gpio_to_slice_num(uGPIO);
    muChan = pwm_gpio_to_channel(uGPIO);
 
     // Configure GPIO for PWM usage
    gpio_set_function(uGPIO, GPIO_FUNC_PWM);
    pwm_set_clkdiv(muSlice, kClockDivider);
    pwm_set_wrap(muChan, kCountMax);

    // Set initial position
    setPosition(fInitialPosition);
 
    // Switch it on
    pwm_set_enabled(muSlice, true);
}

Servo::~Servo() {
    pwm_set_enabled(muSlice, false);
    gpio_set_function(muGPIO, GPIO_FUNC_NULL);
}

bool Servo::isValid(void) const {
    return mbIsValid;
}

void Servo::setPosition(float fPosition) {
    fPosition = (fPosition<0.0f)?0.0f:((fPosition>1.0f)?1.0f:fPosition);    // Constrain range
    if (false == mbLongestPulseIsMax) {
        fPosition = 1.0f - fPosition;
    }
    uint uPWMCount((uint)roundf((fPosition * (float)(muServoCountMax - muServoCountMin))) + muServoCountMin);
    pwm_set_chan_level(muSlice, muChan, uPWMCount);
}
