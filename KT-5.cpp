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


#define kDialServoGPIO  (14)    ///< Dial servo is on GPIO14 (pin 19)

static
float _getServoPosnForKts(float fKts) {
    fKts = ((fKts < 0.0f)?0.0f:(fKts > 8.0f)?8.0f:fKts);    // Constrain in range
    static const float pfPosn[] = {
 //       0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f, 1.0f
        0.0f, 0.17f, 0.30f, 0.425f, 0.58f, 0.70f, 0.82f, 0.92f, 1.0f
    };
    uint uPreIndex((uint)floorf(fKts)), uPostIndex((uint)ceilf(fKts));
    if (uPreIndex == uPostIndex) {
        return pfPosn[uPreIndex];
    }

    // Linear interpolate
    float fAlpha(fKts - (float)uPreIndex);
    float fPosn((fAlpha * pfPosn[uPostIndex]) + ((1.0f - fAlpha)*pfPosn[uPreIndex]));
    return fPosn;
}

int main() {
    stdio_init_all();

    // Create dial servo object
    Servo cDial(kDialServoGPIO, 0.0f, false);

    //  Show we are alive
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    uint uPosn(0);
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




    /*
    // Timer example code - This example fires off the callback after 2000ms
    add_alarm_in_ms(2000, alarm_callback, NULL, false);

    puts("Hello, world!");

    // Tell the LED pin that the PWM is in charge of its value.
    gpio_set_function(PICO_DEFAULT_LED_PIN, GPIO_FUNC_PWM);
    // Figure out which slice we just connected to the LED pin
    uint slice_num = pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config config = pwm_get_default_config();
    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv(&config, 1.0f);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice_num, &config, true);

    // Everything after this point happens in the PWM interrupt handler, so we
    // can twiddle our thumbs
    while (1)
        tight_loop_contents();
        */
