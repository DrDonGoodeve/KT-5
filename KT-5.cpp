/******************************************************************************
 * KT-5.cpp
 *
 * KT-5 Knotmeter head project - main application file.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 30Mar2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

// Includes
//*****************************************************************************
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

#include "Servo.h"


// Defines
//*****************************************************************************
#define kDialServoGPIO  (14)    ///< Dial servo is on GPIO14 (pin 19)


// Local functions
//*****************************************************************************
// The servo is not centered on the dial - and hence to get the digits lining
// up correctly a correction needs to be applied. This function maps a speed
// in Knots to the correct servo position control (where 0.0f corresponds to
// 0 knots and 1.0f corresponds to 8 knots on the dial - also is close to maximum
// deflection for servo. Hull speed is around 6.2 knots...)
//-----------------------------------------------------------------------------
static float _getServoPosnForKts(float fKts) {
    static const float pfPosn[] = {	// Remapping table
    //	0kts	1kt	2kts	3kts	4kts	5kts	6kts	7kts	8kts
        0.0f, 	0.17f, 	0.30f, 	0.425f,	0.58f,	0.70f, 	0.82f,	0.92f,	1.0f
    };

    // Constrain kts parameter to be in valid range
    fKts = ((fKts < 0.0f)?0.0f:(fKts > 8.0f)?8.0f:fKts);


    // Compute table indices
    uint uPreIndex((uint)floorf(fKts)), uPostIndex((uint)ceilf(fKts));
    if (uPreIndex == uPostIndex) {
        return pfPosn[uPreIndex];
    }

    // Linear interpolate for intermediate positions
    float fAlpha(fKts - (float)uPreIndex);
    float fPosn((fAlpha * pfPosn[uPostIndex]) + ((1.0f - fAlpha)*pfPosn[uPreIndex]));
    return fPosn;
}


// Application entry-point
//*****************************************************************************
int main(void) {
    stdio_init_all();

    // Create dial servo object
    Servo cDial(kDialServoGPIO, 0.0f, false);

    //  Show we are alive by driving the PICO LED output
    const uint LED_PIN(PICO_DEFAULT_LED_PIN);
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    uint uPosn(0);

    // Forever...
    while (true) {
        cDial.setPosition(_getServoPosnForKts(0.0f));
        sleep_ms(1000);
        for(uint uKts=0; uKts<8; uKts++) {
            for(uint uFrac=0; uFrac<100; uFrac++) {
                float fKts((float)uKts + ((float)uFrac / 100.0f));
                cDial.setPosition(_getServoPosnForKts(fKts));
                sleep_ms(10);
            }
            printf("Fluffy uKts = %d\r\n", uKts+1);
            gpio_put(LED_PIN, 1);
            sleep_ms(80);
            gpio_put(LED_PIN, 0);
            sleep_ms(100);
            gpio_put(LED_PIN, 1);
            sleep_ms(80);
            gpio_put(LED_PIN, 0);
            sleep_ms(400);        
        }
    }
}

