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
#include "hardware/irq.h"


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
#define kMaxStep        (30)


// Macros
//-----------------------------------------------------------------------------
#define _min(a,b)   ((a)<(b)?(a):(b))
#define _max(a,b)   ((a)>(b)?(a):(b))


// Local variables
//-----------------------------------------------------------------------------
static Servo *mpIRQOwner = nullptr;
uint guServoLine = 0;


// Local functions
//-----------------------------------------------------------------------------
void _pwmIRQ(void) {
    if (mpIRQOwner == nullptr) {
        return;
    }

    pwm_clear_irq(mpIRQOwner->muSlice);

    mpIRQOwner->adjustPWMSetting();
}


// Servo:: implementation
//-----------------------------------------------------------------------------
Servo::Servo(
    uint uGPIO, float fInitialPosition, bool bLongestPulseIsMaxPosition, 
    float fMinPulseSec, float fMaxPulseSec) :
    mbIsValid(false), mbLongestPulseIsMax(bLongestPulseIsMaxPosition),
    muGPIO(uGPIO), muSlice(pwm_gpio_to_slice_num(uGPIO)), muChan(pwm_gpio_to_channel(uGPIO)),
    muTargetPWM(0), muCurrentPWM(0) {

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

    // Provide dereference for IRQ handler
    mpIRQOwner = this;

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(muSlice);
    pwm_set_irq_enabled(muSlice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, _pwmIRQ);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Set initial position - immediate; no tracking
    setPosition(fInitialPosition, false);
 
    // Switch it on
    pwm_set_enabled(muSlice, true);
}

Servo::~Servo() {
    mpIRQOwner = nullptr;
    pwm_set_irq_enabled(muSlice, false);
    pwm_set_enabled(muSlice, false);
    gpio_set_function(muGPIO, GPIO_FUNC_NULL);
}

bool Servo::isValid(void) const {
    return mbIsValid;
}

// Set servo position - if bTracking is false, move immediately
// otherwise smoothly track to target position.
//-----------------------------------------------------------------------------
void Servo::setPosition(float fPosition, bool bTracking) {
    fPosition = (fPosition<0.0f)?0.0f:((fPosition>1.0f)?1.0f:fPosition);    // Constrain range
    if (false == mbLongestPulseIsMax) {
        fPosition = 1.0f - fPosition;
    }
    uint uPWMCount((uint)roundf((fPosition * (float)(muServoCountMax - muServoCountMin))) + muServoCountMin);
    if (false == bTracking) {
        muCurrentPWM = muTargetPWM = uPWMCount;
        pwm_set_chan_level(muSlice, muChan, uPWMCount);
    } else {
        muTargetPWM = uPWMCount;
    }
}

// Set the PWM to the next value tracking towards muTargetPWM
// from muCurrentPWM
//-----------------------------------------------------------------------------
void Servo::adjustPWMSetting(void) {
    guServoLine = __LINE__;
    if (muTargetPWM < muCurrentPWM) {
    guServoLine = __LINE__;
        uint16_t uDiff(muCurrentPWM - muTargetPWM);
        uint16_t uDelta(_min(uDiff, kMaxStep));
        muCurrentPWM -= uDelta;
    } else if (muTargetPWM > muCurrentPWM) {
     guServoLine = __LINE__;
        uint16_t uDiff(muTargetPWM - muCurrentPWM);
        uint16_t uDelta(_min(uDiff, kMaxStep));
        muCurrentPWM += uDelta;
    }
    pwm_set_chan_level(muSlice, muChan, muCurrentPWM);
}
