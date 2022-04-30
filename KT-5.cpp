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

volatile uint guKT5Line = 0;
extern uint guOLEDLine;
extern uint guADCLine;

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
guKT5Line = __LINE__;
    sbDisplayUpdateDue = true;  // Picked up in main loop
    if (0 == suTimersBeforeDisplayChange) {
        suTimersBeforeDisplayChange = kDisplayUpdatePeriod / kDisplayUpdateTimerPeriod;
        seDisplayCycle = (kDisplayDistance==seDisplayCycle)?kDisplaySpeed:(_displayCycle)((int)seDisplayCycle+1);
    } else {
        suTimersBeforeDisplayChange--;
    }
guKT5Line = __LINE__;
    return true;
}

// Command processor running on second core (uses blocking calls - cheap thread)
// Waits (blocking) on stdin. On receipt of a command attempts to interpret it. If
// successfully interpreted, encodes as a _Command which can be picked up
// (non-blocking) by the main loop via _getCommand.
typedef enum {
    kNoOp,           // Null operation
    kIgnore,         // Ignore the command
    kHelp,           // Help command
    kInput,          // Report raw input
    kReportKts,     // <kts> report input mapped to kts
    kSetKtsVal,     // <kts> <input> set kts map to input
    kReportServoKts, // <kts> report posn mapped to kts
    kSetServoKts,    // <kts> <posn> set kts map to posn
    kServoOn,        // servo response on
    kServoOff,       // servo response off
    kServoPos,       // set servo position
    kSave,           // save settings to flash
    kRestart         // Force restart
}OpCode;

static const struct {
    const char *pTag;
    OpCode eCode;
    uint uArgs;
    const char *pInfo;
}spCommands[] = {
    {"?", kHelp, 0, "- show help and diagnostics"},
    {"raw", kInput, 0, "- display current raw transducer signal"},
    {"kts", kReportKts, 0, "- show mappings of kts to raw transducer signal"},
    {"setkts", kSetKtsVal, 2, "<kts> <raw> - set mapping from raw value to kts"},
    {"servokts", kReportServoKts, 0, "- show mappings of kts to servo posn"},
    {"setservo", kSetServoKts, 2, "<kts> <posn> - set mapping of kts to servo posn"},
    {"servoauto", kServoOn, 0, "- switch servo to auto (default)"},
    {"servoman", kServoOff, 0, "- switch servo to manual"},
    {"servopos", kServoPos, 1, "<posn> - set servo posn (manual mode only)"},
    {"save", kSave, 0, "- save all settings to flash"},
    {"restart", kRestart, 0, "- restart KT-5"}
};

class _Command {
    public:
        uint8_t muOpCode;
        uint8_t muData1;
        uint8_t muData2;
    
        _Command(void) : 
            muOpCode(kNoOp), muData1(0x0), muData2(0x0) {
        }

        _Command(const std::string &sInput) :
            muOpCode(kNoOp), muData1(0x0), muData2(0x0) {
            
            printf("KT5:%d, OLED:%d, ADC:%d\r\n", guKT5Line, guOLEDLine, guADCLine);

            /* temporary disable
            std::string sTag(sInput);
            size_t iSpace(sInput.find(" "));
            if (iSpace != std::string::npos) {
                sTag = sInput.substr(0, iSpace);
                std::string sArgs(sInput.substr(iSpace+1));
                iSpace = sArgs.find(" ");
                if (std::string::npos == iSpace) {
                    muData1 = stoi(sArgs);
                    //printf("sArg1 = %s\r\n", sArgs.c_str());
                } else {
                    std::string sArg1(sArgs.substr(0, iSpace));
                    std::string sArg2(sArgs.substr(iSpace+1));
                    //printf("sArg1 = %s, sArg2 = %s\r\n", sArg1.c_str(), sArg2.c_str());
                    muData1 = stoi(sArg1);
                    muData2 = stoi(sArg2);
                }
            }

            trim(sTag);
            for(uint i=0; i<_arraysize(spCommands); i++) {
                if (0 == sTag.compare(spCommands[i].pTag)) {
                    muOpCode = spCommands[i].eCode;
                    break;
                }
            }
            if (kHelp == muOpCode) {  // Help command
                muOpCode = kIgnore; // Always runs on core1 - aid with debugging
                std::string sHelp("command:\r\n");
                for(uint i=0; i<_arraysize(spCommands); i++) {
                    sHelp += std::string("\t") + std::string(spCommands[i].pTag) + std::string(" ") + std::string(spCommands[i].pInfo) + std::string("\r\n");
                }
                printf(sHelp.c_str());
                printf("KT5:%d, OLED:%d, ADC:%d\r\n", guKT5Line, guOLEDLine, guADCLine);
            } else if (kRestart == muOpCode) {
                muOpCode = kIgnore;
                printf("Restarting...");
                // add restert code here
            }
            */
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

        bool canSend(void) {
            return ((muOpCode != kNoOp) && (muOpCode != kIgnore) && (muOpCode != kHelp) && (muOpCode != kRestart));
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
        if (true == cCommand.canSend()) {
            multicore_fifo_push_blocking(cCommand.getAs32Bit());
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
    OLED *pDisplay(new OLED());
    pDisplay->show(kAppInfo1);
    sleep_ms(500);

    // Setup connection to HC-05
    HC05 *pBluetooth(new HC05("KT-5"));

    for(uint i=128; i>6; i--) {
        char pBuffer[64];
        sprintf(pBuffer, kAppInfo1 kAppInfo2, i);
        pDisplay->show(pBuffer);
        sleep_ms(5);
    }

    // Create dial servo object and zero position
    Servo *pDial(new Servo(kDialServoGPIO, 0.0f, false));
    pDial->setPosition(1.0f);
    sleep_ms(500);
    pDial->setPosition(0.0f);
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
    SpeedMeasurement *pMeasure(new SpeedMeasurement(kSampleRateHz));

    // Create ADCEngine and start sampling
    ADCEngine *pADC(new ADCEngine(kADCChannel, kSampleRateHz, kADCBuffer, kADCFrames));
    pADC->setActive(true);

    // Setup display update rollover timer
    add_repeating_timer_ms(kDisplayUpdateTimerPeriod, _displayUpdateCallback, nullptr, &mcDisplayTimer);

    // Announce application and start command processor
    printf("KT-5 Application\r\n");
    multicore_launch_core1(_commandProcessor);

#define _FULLAPP
#ifdef _FULLAPP

    // Main loop
    while(true) {
guKT5Line = __LINE__;
        while(false == sbDisplayUpdateDue) {
guKT5Line = __LINE__;
            if (false == pADC->processFrame(pMeasure)) {
                //printf(".");
                sleep_ms(kProcessingPause);
            } else {
                // Update dial every time a new measurement is made...
                float fKts(pMeasure->getSpeedKts());
                pDial->setPosition(_getServoPosnForKts(fKts));               
            }
guKT5Line = __LINE__;
        }
        sbDisplayUpdateDue = false;
        
        // Blink status
guKT5Line = __LINE__;
        gpio_put(PICO_DEFAULT_LED_PIN, (true == bAliveLEDOn)?0:1);
        bAliveLEDOn = !bAliveLEDOn;

        // Process any command received
guKT5Line = __LINE__;
        _Command cCommand;
        if (true == _getCommand(cCommand)) {
            //printf("Command %d, a1:%d, a2:%d\r\n", cCommand.muOpCode, cCommand.muData1, cCommand.muData2);
        }

        // Update the display
guKT5Line = __LINE__;
        char pBuffer[64];
        switch(seDisplayCycle) {
            case kDisplaySpeed: {
                float fKts(pMeasure->getSpeedKts());
                sprintf(pBuffer, "@(4,40,-6)%.1f@(2,10,34)K N O T S", fKts); 
                break;
            }
            case kDisplayTime: {
               uint uSec((uint)roundf(pMeasure->getSecondsElapsed()));
                uint uHrs(uSec / 3600); uSec -= (uHrs*3600);
                uint uMin(uSec / 60); uSec -= (uMin*60);
                sprintf(pBuffer, "@(3,0,-4)%02d:%02d:%02d@(2,10,34)ELAPSED", uHrs, uMin, uSec);
                break;
            }
            case kDisplayAvgSpeed: {
                float fKtsAvg(pMeasure->getAvgSpeedKts());
                sprintf(pBuffer, "@(4,16,-6)%.3f@(2,20,34)avg KTS", fKtsAvg);
                break;
            }
            case kDisplayDistance: default: {
                float fNM(pMeasure->getDistanceTravelledNm());
                sprintf(pBuffer, "@(4,30,-6)%.2f@(2,20,34)SEA nm", fNM);
                break;
            }
        }

guKT5Line = __LINE__;
        std::string sDisplay(pBuffer);
guKT5Line = __LINE__;
        pDisplay->show(sDisplay);
guKT5Line = __LINE__;
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