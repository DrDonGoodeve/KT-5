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
#include <iostream>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"

#include "Servo.h"
#include "ADCEngine.h"
#include "SpeedMeasurement.h"
#include "OLED.h"
#include "HC-05.h"


// Defines
//*****************************************************************************
#define kDialServoGPIO          (14)   ///< Dial servo is on GPIO14 (pin 19)
#define kAppInfo1               "@(4,16,-6)KT-5" 
#define kAppInfo2               "@(2,%d,32)reloaded"
#define kADCChannel             (0)
#define kSampleRateHz           (10.0e3f)
#define kADCBuffer              (500.0e-3f)
#define kADCFrames              (4)
#define kProcessingPause        (50)
#define kDisplayUpdatePeriod        (2000)
#define kDisplayUpdateTimerPeriod   (200)


// Local types
//*****************************************************************************
// Display sequence - every 4 seconds changes
typedef enum _displayCycle {
    kDisplaySpeed = 0, 
    kDisplayTime = 1, 
    kDisplayAvgSpeed = 2, 
    kDisplayDistance = 3
}DisplayCycle;


// Local variables
//*****************************************************************************
static bool sbDisplayUpdateDue(false);
static DisplayCycle seDisplayCycle(kDisplaySpeed);
static uint suTimersBeforeDisplayChange(0);

repeating_timer_t mcDisplayTimer;


// Local functions, classes
//*****************************************************************************
// The servo is not centered on the dial - and hence to get the digits lining
// up correctly a correction needs to be applied. This function maps a speed
// in Knots to the correct servo position control (where 0.0f corresponds to
// 0 knots and 1.0f corresponds to 8 knots on the dial - also is close to maximum
// deflection for servo. Hull speed is around 6.2 knots...)
//-----------------------------------------------------------------------------
static float _getServoPosnForKts(float fKts) {
    static const float pfPosn[] = {	// Remapping table
    //	0kts	1kt	    2kts	3kts	4kts	5kts	6kts	7kts	8kts
        0.0f, 	0.17f, 	0.32f, 	0.435f,	0.58f,	0.70f, 	0.83f,	0.93f,	1.0f
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

// Called on timer every kDisplayUpdatePeriod msec - triggers display update
bool _displayUpdateCallback(repeating_timer_t *pTimer) {
    //printf("_displayUpdateCallback\r\n");
    sbDisplayUpdateDue = true;  // Picked up in main loop
    if (0 == suTimersBeforeDisplayChange) {
        suTimersBeforeDisplayChange = kDisplayUpdatePeriod / kDisplayUpdateTimerPeriod;
        seDisplayCycle = (kDisplayDistance==seDisplayCycle)?kDisplaySpeed:(_displayCycle)((int)seDisplayCycle+1);
    } else {
        suTimersBeforeDisplayChange--;
    }
    return true;
}

// Command processor running on second core (uses blocking calls - cheap thread)
// Waits (blocking) on stdin. On receipt of a command attempts to interpret it. If
// successfully interpreted, encodes as a _Command which can be picked up
// (non-blocking) by the main loop via _getCommand.
class _Command {
    public:
        uint8_t muOpCode;
        uint8_t muData1;
        uint8_t muData2;

        enum {
            kNoOp,           // Null operation
            kHelp,           // Help command
            kInput,          // Report raw input
            kReportCalSpeed, // <kts> report input mapped to kts
            kSetCalSpeed,    // <kts> <input> set kts map to input
            kReportCalServo, // <kts> report posn mapped to kts
            kSetCalServo,    // <kts> <posn> set kts map to posn
            kServoOn,        // servo response on
            kServoOff,       // servo response off
            kSave,           // save settings to flash
            kRestart         // Force restart
        }OpCodes;
    
        _Command(void) : 
            muOpCode(kNoOp), muData1(0x0), muData2(0x0) {
        }

        _Command(const std::string &sInput) {
            if (0 == sInput.compare(0, 1, "?")) {  // Help command
                muOpCode = kNoOp;
                printf("commands:\r\n"
                    "\t? - help (this command)\r\n"
                    "\tinput - report current uncalibrated speed value\r\n"
                    "\trspd <kts> - report speed value associated with kts\r\n"
                    "\tsspd <kts> <spdval> - associate kts with speed value\r\n"
                    "\tcal <num> - apply adjustment from 0 to 255 (default 128)\r\n"
                    "\tservo <num> - switch servo response off (0) or on (otherwise)\r\n"
                    "\tsposk <num> - move servo to kts posn 0..8\r\n"
                    "\tspos <num> - move servo to absolute posn 0..255\r\n"
                    "\tsave - save settings to flash\r\n\n"
                );

            } else if (0 == sInput.compare(0, 3, "cal")) { // Adjust command
                muOpCode = kReportCalSpeed;
                std::string sCode(sInput.substr(4));
                muData1 = stoi(sCode);

            } else if (0 == sInput.compare(0, 4, "save")) { // Save command
                muOpCode = kSave;
            
            } else if (0 == sInput.compare(0, 3, "son")) { // Servo on/off
                muOpCode = kServoOn;

            } else if (0 == sInput.compare(0, 4, "soff")) { // Servo on/off
                muOpCode = kServoOn;
            }
        }

        _Command(uint32_t uCode) :
            muOpCode(uCode&0xff), muData1((uCode>>8)&0xff), muData2((uCode>>16)&0xff) {
        }

        _Command &operator=(const _Command &cOther) {
            muOpCode = cOther.muOpCode;
            muData1 = cOther.muData1;
            muData2 = cOther.muData2;
            return *this;
        }

        bool isValid(void) {
            return (muOpCode != kNoOp);
        }

        uint32_t getAs32Bit(void) {
            return (((uint32_t)muOpCode) | ((uint32_t)muData1<<8) | ((uint32_t)muData2<<16));
        }
};

// Blocking loop - launch on core 1
void _commandProcessor(void) {
    printf("Waiting for command...\r\n");
    
    while(true) {
        std::string sInput;
        std::getline(std::cin, sInput); // Blocking
        // Interpret
        _Command cCommand(sInput);
        if (true == cCommand.isValid()) {
            multicore_fifo_push_blocking(cCommand.getAs32Bit());
        } else {
            printf("Unknown command '%s' - use '?' for help.\r\n", sInput.c_str());
        }
    }
}

bool _getCommand(_Command &cCommand) {
    uint32_t uCmd(0x0);
    if (false == multicore_fifo_rvalid()) {
        return false;
    }
    if (false == multicore_fifo_pop_timeout_us(10LL, &uCmd)) {
        return false;
    }
    
    cCommand = _Command(uCmd);
    return true;
}


// Application entry-point
//*****************************************************************************
int main(void) {
    stdio_init_all();

    // Startup display
    OLED cDisplay;
    char pBuffer[64];
    cDisplay.show(kAppInfo1);
    sleep_ms(500);

    // Setup connection to HC-05
    HC05 cBluetooth("KT-5");

    for(uint i=128; i>6; i--) {
        sprintf(pBuffer, kAppInfo1 kAppInfo2, i);
        cDisplay.show(pBuffer);
        sleep_ms(5);
    }

    // Create dial servo object and zero position
    Servo cDial(kDialServoGPIO, 0.0f, false);
    cDial.setPosition(1.0f);
    sleep_ms(500);
    cDial.setPosition(0.0f);
    sleep_ms(2000);

    // Alive LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    bool bAliveLEDOn(false);

    // 10 second warning to get UART connection (testing)
    for(uint i=0; i<10; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(20);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(980);
    }

    // Create measurement object
    SpeedMeasurement cMeasure(kSampleRateHz);

    // Create ADCEngine and start sampling
    ADCEngine cADC(kADCChannel, kSampleRateHz, kADCBuffer, kADCFrames);
    cADC.setActive(true);

    // Setup display update rollover timer
    add_repeating_timer_ms(kDisplayUpdateTimerPeriod, _displayUpdateCallback, nullptr, &mcDisplayTimer);

    // Announce application and start command processor
    printf("KT-5 Application\r\n");
    multicore_launch_core1(_commandProcessor);

#define _FULLAPP
#ifdef _FULLAPP

    // Main loop
    while(true) {
        while(false == sbDisplayUpdateDue) {
            if (false == cADC.processFrame(cMeasure)) {
                //printf(".");
                sleep_ms(kProcessingPause);
            } else {
                // Update dial every time a new measurement is made...
                float fKts(cMeasure.getSpeedKts());
                cDial.setPosition(_getServoPosnForKts(fKts));               
            }
        }
        sbDisplayUpdateDue = false;
        
        // Blink status
        gpio_put(PICO_DEFAULT_LED_PIN, (true == bAliveLEDOn)?0:1);
        bAliveLEDOn = !bAliveLEDOn;

        // Process any command received
        _Command cCommand;
        if (true == _getCommand(cCommand)) {
            if (_Command::kReportCalSpeed == cCommand.muOpCode) {
                printf("Execute: kReportCalSpeed with data %d\r\n", cCommand.muData1);
            } else if (_Command::kSave == cCommand.muOpCode) {
                printf("Execute: Save settings to flash\r\n");
            }
        }

        // Update the display
        switch(seDisplayCycle) {
            case kDisplaySpeed: {
                float fKts(cMeasure.getSpeedKts());
                sprintf(pBuffer, "@(4,40,-6)%.1f@(2,10,34)K N O T S", fKts); 
                break;
            }
            case kDisplayTime: {
                uint uSec((uint)roundf(cMeasure.getSecondsElapsed()));
                uint uHrs(uSec / 3600); uSec -= (uHrs*3600);
                uint uMin(uSec / 60); uSec -= (uMin*60);
                sprintf(pBuffer, "@(3,0,-4)%02d:%02d:%02d@(2,10,34)ELAPSED", uHrs, uMin, uSec);
                break;
            }
            case kDisplayAvgSpeed: {
                float fKtsAvg(cMeasure.getAvgSpeedKts());
                //uint uKtsAvg((uint)floorf(fKtsAvg));
                //sprintf(pBuffer, "@(4,16,-6)%.3f@(2,20,34)avg KTS", uKtsAvg, (uint)roundf(((fKtsAvg-(float)uKtsAvg)*1000.0f)));
                sprintf(pBuffer, "@(4,16,-6)%.3f@(2,20,34)avg KTS", fKtsAvg);
                break;
            }
            case kDisplayDistance: default: {
                float fNM(cMeasure.getDistanceTravelledNm());
                sprintf(pBuffer, "@(4,30,-6)%.2f@(2,20,34)SEA nm", fNM);
                break;
            }
        }

        cDisplay.show(pBuffer);
    }
}

#else // _FULLAPP
    //  Show we are alive by driving the PICO LED output
    // Forever...
    char pBuffer[32];
    uint uStep(0);
    while (true) {
        cDial.setPosition(_getServoPosnForKts(0.0f));
        sleep_ms(500);
        for(uint uKts=0; uKts<8; uKts++) {
            for(uint uFrac=0; uFrac<=100; uFrac++) {
                float fKts((float)uKts + ((float)uFrac / 100.0f));
                cDial.setPosition(_getServoPosnForKts(fKts));
                sleep_ms(1);
                if (0 == (uFrac % 10)) {
                    switch (uStep) {
                        //sprintf(pBuffer, "@(4,12,-6)%.1fkts", fKts);
                        case 0: sprintf(pBuffer, "@(4,40,-6)%.1f@(2,10,34)K N O T S", fKts); break;
                        case 1: sprintf(pBuffer, "@(4,30,-6)%.2f@(2,20,34)SEA nm", fKts); break;
                        case 2: sprintf(pBuffer, "@(3,0,-4)%02d:%02d:%02d@(2,10,34)ELAPSED", (int)(fKts*10.0f), (int)(fKts*10.0f), (int)(fKts*10.0f)); break;
                        case 3: default: sprintf(pBuffer, "@(4,16,-6)%.3f@(2,20,34)avg KTS", fKts); break;
                    }
                    cDisplay.show(pBuffer);
                }
            }
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(80);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(80);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(1000);        
        }
        uStep = ((uStep+1)%4);
    }
}
#endif // _FULLAPP