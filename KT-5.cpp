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

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

void on_pwm_wrap() {
    static int fade = 0;
    static bool going_up = true;
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN));

    if (going_up) {
        ++fade;
        if (fade > 255) {
            fade = 255;
            going_up = false;
        }
    } else {
        --fade;
        if (fade < 0) {
            fade = 0;
            going_up = true;
        }
    }
    // Square the fade value to make the LED's brightness appear more linear
    // Note this range matches with the wrap value
    pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, fade * fade);
}

#define kServoPWMGPIO   (14)
#define kServoPWMFreq   (50.0f)
#define kServoPWMPeriod (1.0f / kServoPWMFreq)
//#define kServoPWMMin    (900.0e-6f)
//#define kServoPWMMax    (2100.0e-6f)
#define kServoPWMMin    (470.0e-6f)
#define kServoPWMMax    (2650.0e-6f)

#define kPicoClock      (125.0e6f)
#define kPWMCountMax    (0x10000)
#define kPWMFullFreq    (125.0e6f / (float)kPWMCountMax)

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
#define kServoCountMin  ((uint)roundf((kServoPWMMin / kExactPeriod) * (float)kCountMax))
#define kServoCountMax  ((uint)roundf((kServoPWMMax / kExactPeriod) * (float)kCountMax))

#define kTestPosn       (1.0f)
#define kTestCount      ((uint)roundf(((1.0f - kTestPosn) * (float)(kServoCountMax - kServoCountMin))) + kServoCountMin)

static
void _setServoPosn(float fPosn, uint uSlice, uint uChan) {
    uint uPWMCount((uint)roundf(((1.0f - fPosn) * (float)(kServoCountMax - kServoCountMin))) + kServoCountMin);
    pwm_set_chan_level(uSlice, uChan, uPWMCount);
}

static
float _getServoPosnForKts(float fKts) {
    fKts = ((fKts < 0.0f)?0.0f:(fKts > 8.0f)?8.0f:fKts);    // Constrain in range
    static const float pfPosn[] = {
 //       0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f, 1.0f
        0.0f, 0.16f, 0.30f, 0.425f, 0.58f, 0.70f, 0.82f, 0.92f, 1.0f
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

    // Configure GPIO kServoPWM
    gpio_set_function(kServoPWMGPIO, GPIO_FUNC_PWM);
    uint uSSlice(pwm_gpio_to_slice_num(kServoPWMGPIO));
    uint uSChan(pwm_gpio_to_channel(kServoPWMGPIO));
    pwm_set_clkdiv(uSSlice, kClockDivider);
    pwm_set_wrap(uSSlice, kCountMax);

    // Set a test value
    pwm_set_chan_level(uSSlice, uSChan, kTestCount);

    // Switch it on
    pwm_set_enabled(uSSlice, true);

    //  Show we are alive
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    uint uPosn(0);
    while (true) {
        _setServoPosn(_getServoPosnForKts(0.0f), uSSlice, uSChan);
        sleep_ms(1000);
        for(uint uKts=0; uKts<8; uKts++) {
            printf("uKts = %d\r\n", uKts);
            for(uint uFrac=0; uFrac<100; uFrac++) {
                float fKts((float)uKts + ((float)uFrac / 100.0f));
                _setServoPosn(_getServoPosnForKts(fKts), uSSlice, uSChan);
                sleep_ms(10);
            }
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
