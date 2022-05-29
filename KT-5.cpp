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
#include "hardware/watchdog.h"
#include "pico/multicore.h"

#include "Servo.h"
#include "ADCEngine.h"
#include "SpeedMeasurement.h"
#include "OLED.h"
#include "HC-05.h"
#include "Flash.h"


// Defines
//*****************************************************************************
// Application information
#define kAppName                "KT-5"
#define kVersion                "B.8"
#define kAppInfo1               "@(4,16,-6)" kAppName
#define kSlidePrefix            "@(2,%d,32)"
#define kSlideAppInfo2          kSlidePrefix "reloaded..."
#define kSlideVersion           kSlidePrefix "version " kVersion
#define kKT5SettingsSignature   (0x7155e771)

// ADC sampling control
#define kADCChannel                 (2)
#define kSampleRateHz               (10.0e3f)
#define kADCBuffer                  (500.0e-3f)
#define kADCFrames                  (4)

// Display control
#define kProcessingPause            (50)
#define kDisplayUpdatePeriod        (2000)
#define kDisplayUpdateTimerPeriod   (200)

// Servo control
#define kDialServoGPIO          (14)   ///< Dial servo is on GPIO14 (pin 19)
#define kDefaultServoRate       (30)
#define kDefaultServoMin        (18)
#define kDefaultServoMax        (235)

// Settings information
#define kKtsPositions   (9)
#define kSettingsMagic  (0x5e771235)


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
extern uint guServoLine;


// Local variables
//*****************************************************************************
static bool sbDisplayUpdateDue(false);
static DisplayCycle seDisplayCycle(kDisplaySpeed);
static uint suTimersBeforeDisplayChange(0);

repeating_timer_t mcDisplayTimer;


// Local functions, classes
//*****************************************************************************
// Settings - stored in flash
// Examines flash for valid settings signature. If found - loads the settings.
// supports restore and save operations.
//-----------------------------------------------------------------------------
class Settings {
    public:
        uint32_t muMagic;                           // Magic number
        uint8_t mpServoKts[kKtsPositions];          // Map integer kts to servo position
        uint8_t mpKtsForReading[kKtsPositions];     // Map reading to kts
        uint8_t muServoRate;                        // Servo tracking rate (default 30)
        uint8_t muServoMin;                         // In 100s of us minus 300 (default 18 for 480us)
        uint8_t muServoMax;                         // In 100s of us minus 300 (default 235 for 2650us)
}scSettings = {
    kSettingsMagic,
    {0, 1*(256/8), 2*(256/8), 3*(256/8), 4*(256/8), 5*(256/8), 6*(256/8), 7*(256/8), 255},
    {0, 1*(256/8), 2*(256/8), 3*(256/8), 4*(256/8), 5*(256/8), 6*(256/8), 7*(256/8), 255},
    kDefaultServoRate,
    kDefaultServoMin,
    kDefaultServoMax
};

// The servo is not centered on the dial - and hence to get the digits lining
// up correctly a correction needs to be applied. This function maps a speed
// in Knots to the correct servo position control (where 0.0f corresponds to
// 0 knots and 1.0f corresponds to 8 knots on the dial - also is close to maximum
// deflection for servo. Hull speed is around 6.2 knots...)
//-----------------------------------------------------------------------------
static float _getServoPosnForKts(float fKts) {
    //static const float pfPosn[] = {	// Remapping table
    ////	0kts	1kt	    2kts	3kts	4kts	5kts	6kts	7kts	8kts
    //    0.0f, 	0.17f, 	0.32f, 	0.435f,	0.58f,	0.70f, 	0.83f,	0.93f,	1.0f
    //};

    // Constrain kts parameter to be in valid range
    fKts = ((fKts < 0.0f)?0.0f:(fKts > 8.0f)?8.0f:fKts);

    // Compute table indices
    uint uPreIndex((uint)floorf(fKts)), uPostIndex((uint)ceilf(fKts));
    float fPos1((float)scSettings.mpServoKts[uPreIndex]/255.0f), fPos2(scSettings.mpServoKts[uPostIndex]/255.0f);

    // Linear interpolate for intermediate positions
    float fAlpha(fKts - (float)uPreIndex);
    float fPosn((fAlpha * fPos2) + ((1.0f - fAlpha)*fPos1));
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
//-----------------------------------------------------------------------------
typedef enum {
    kNoOp,           // Null operation
    kIgnore,         // Ignore the command
    kHelp,           // Help command
    kDiag,           // Display diagnostics
    kInput,          // Report raw input
    kReportKts,      // Report input map to kts
    kSetKtsVal,      // <kts> <input> set kts map to input
    kReportServoKts, // Report servo posn map to kts
    kSetServoKts,    // <kts> <posn> set kts map to posn
    kServoOn,        // servo response on
    kServoPos,       // set servo position
    kServoTime,      // set servo min and max pulse
    kServoRate,      // set servo tracking rate
    kReportADC,      // report captured data
    kSave,           // save settings to flash
    kResetFlash,     // reset flash
    kRestart         // Force restart
}OpCode;

static const struct {
    const char *pTag;
    OpCode eCode;
    uint uArgs;
    const char *pInfo;
}spCommands[] = {
    {"?", kHelp, 0, "- show help and diagnostics"},
    {"d", kDiag, 0, "- display diagnostic information"},
    {"raw", kInput, 0, "- display current raw transducer signal"},
    {"kts", kReportKts, 0, "- show mappings of kts to raw transducer signal"},
    {"setkts", kSetKtsVal, 2, "<kts> <raw> - set mapping from raw value to kts"},
    {"serkts", kReportServoKts, 0, "- show mappings of kts to servo posn"},
    {"setsp", kSetServoKts, 2, "<kts> <posn> - set mapping of kts to servo posn"},
    {"sauto", kServoOn, 0, "- switch servo to auto (default)"},
    {"spos", kServoPos, 1, "<posn> - set servo auto tracking off and move to posn"},
    {"stime", kServoTime, 2, "<min> <max> servo pulse in units of 10us - add 300us"},
    {"srate", kServoRate, 1, "<rate> - set servo tracking rate (1-255)"},
    {"radc", kReportADC, 1, "<centisec> - report centisec of ADC samples"},
    {"save", kSave, 0, "- save all settings to flash"},
    {"rsf", kResetFlash, 2, "92 113 - if codes are entered correctly, erase flash"},
    {"rs", kRestart, 0, "- restart KT-5"}
};

// _Command class
// Encapsulates command parser, encode as 32-bits and decode from 32-bits
//-------------------------------------
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

            std::string sTag(sInput);
            size_t iSpace(sInput.find(" "));
            int iArgs(0);
            if (iSpace != std::string::npos) {
                sTag = sInput.substr(0, iSpace);
                std::string sArgs(sInput.substr(iSpace+1));
                iSpace = sArgs.find(" ");
                if (std::string::npos == iSpace) {
                    muData1 = stoi(sArgs);
                    iArgs = 1;
                    //printf("sArg1 = %s\r\n", sArgs.c_str());
                } else {
                    std::string sArg1(sArgs.substr(0, iSpace));
                    std::string sArg2(sArgs.substr(iSpace+1));
                    //printf("sArg1 = %s, sArg2 = %s\r\n", sArg1.c_str(), sArg2.c_str());
                    muData1 = stoi(sArg1);
                    muData2 = stoi(sArg2);
                    iArgs = 2;
                }
            }

            trim(sTag);
            bool bFound(false);
            for(uint i=0; i<_arraysize(spCommands); i++) {
                if (0 == sTag.compare(spCommands[i].pTag)) {
                    bFound = true;
                    muOpCode = spCommands[i].eCode;
                    if (iArgs != spCommands[i].uArgs) {
                        printf("error: command '%s' takes %d arguments:\r\n\t%s\r\n", spCommands[i].pTag, spCommands[i].uArgs, spCommands[i].pInfo);
                        muOpCode = kIgnore;
                    }
                    break;
                }
            }
            if (false == bFound) {
                printf("Unknown command '%s'\r\n", sTag.c_str());
                return;
            }

            // Handle here (return kIgnore) or return valid command code?
            switch(muOpCode) {
                case kHelp:
                    muOpCode = kIgnore; // Always runs on core1 - aid with debugging
                    printf(kAppName " version " kVersion "- commands:\r\n");
                    for(uint i=0; i<_arraysize(spCommands); i++) {
                        printf("\t%s %s\r\n", spCommands[i].pTag, spCommands[i].pInfo);
                    }
                    break;

                case kRestart:
                    muOpCode = kIgnore;
                    printf("Restarting...");
                    watchdog_enable(1, 1);
                    while(true) {
                    }
                    break;

                case kDiag:
                    muOpCode = kIgnore;
                    printf("KT5:%d, OLED:%d, ADC:%d, Servo:%d\r\n", guKT5Line, guOLEDLine, guADCLine, guServoLine);
                    break;

                default:
                    break;
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

        bool canSend(void) {
            return ((muOpCode != kNoOp) && (muOpCode != kIgnore));
        }

        uint32_t getAs32Bit(void) {
            return (((uint32_t)muOpCode) | ((uint32_t)muData1<<8) | ((uint32_t)muData2<<16));
        }
};

// Blocking loop - launch on core 1
//-------------------------------------
static bool sbFirstRun = true;
static void _commandProcessor(void) {
    if (true == sbFirstRun) {
        sbFirstRun = false;
        printf("KT-5 " kVersion " running\r\n(waiting for command...)\r\n");
    }
    
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

// Poll for command from FIFO
//-------------------------------------
static bool _getCommand(_Command &cCommand) {
    if (false == multicore_fifo_rvalid()) {
        return false;
    }

    uint32_t uCmd(0x0);
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
    cDisplay.show(kAppInfo1);
    sleep_ms(500);

    // Setup connection to HC-05
    HC05 cBluetooth;

    // Banner display
    guKT5Line = __LINE__;
    for(uint i=128; i>2; i--) {
        char pBuffer[64];
        sprintf(pBuffer, kAppInfo1 kSlideAppInfo2, i);
        cDisplay.show(pBuffer);
        sleep_ms(5);
    }
    sleep_ms(400);
    for(uint i=128; i>0; i--) {
        char pBuffer[64];
        sprintf(pBuffer, kAppInfo1 kSlideVersion, i);
        cDisplay.show(pBuffer);
        sleep_ms(5);
    }

    // Create dial servo object and zero position
    guKT5Line = __LINE__;
    float fMinPulse(((float)scSettings.muServoMin * 10.0e-6f) + 300.0e-6f);
    float fMaxPulse(((float)scSettings.muServoMax * 10.0e-6f) + 300.0e-6f);
    printf("min = %.6f, max = %.6f\r\n", fMinPulse, fMaxPulse);
    Servo cDial(kDialServoGPIO, 1.0f, false, fMinPulse, fMaxPulse);
    cDial.setRate(scSettings.muServoRate);
    sleep_ms(400);
    cDial.setPosition(0.0f, false);
    sleep_ms(400);

    // Open flash memory archive and retrieve current settings (if present)
    Flash cFlash;
    uint8_t uSize(0);
    const uint8_t *pFlashSettings(cFlash.readBlock(kKT5SettingsSignature, uSize));
    if ((pFlashSettings != nullptr) && (uSize == sizeof(scSettings))) {
        printf("DEBUG: Found correct size structure\r\n");
        memcpy((void*)&scSettings, pFlashSettings, sizeof(scSettings));
    }

    // Alive LED
    guKT5Line = __LINE__;
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    bool bAliveLEDOn(false);

    // Launch command processor on core1
    guKT5Line = __LINE__;
    multicore_launch_core1(_commandProcessor);

    // Create measurement object
    guKT5Line = __LINE__;
    SpeedMeasurement cMeasure(kSampleRateHz);

    // Create ADCEngine and start sampling
    guKT5Line = __LINE__;
    ADCEngine cADC(kADCChannel, kSampleRateHz, kADCBuffer, kADCFrames);
    guKT5Line = __LINE__;
    cADC.setActive(true);

    // Setup display update rollover timer
    add_repeating_timer_ms(kDisplayUpdateTimerPeriod, _displayUpdateCallback, nullptr, &mcDisplayTimer);

    // Main loop
    bool bServoAuto(true);
    while(true) {
guKT5Line = __LINE__;
        while(false == sbDisplayUpdateDue) {
guKT5Line = __LINE__;
            if (false == cADC.processFrame(cMeasure)) {
                //printf(".");
                sleep_ms(kProcessingPause);
            } else {
                if (true == bServoAuto) {
                    // Update dial every time a new measurement is made...
                    float fKts(cMeasure.getSpeedKts());
                    cDial.setPosition(_getServoPosnForKts(fKts));
                }        
            }
guKT5Line = __LINE__;
        }
        sbDisplayUpdateDue = false;
        
        // Blink status
guKT5Line = __LINE__;
        gpio_put(PICO_DEFAULT_LED_PIN, (true == bAliveLEDOn)?0:1);
        bAliveLEDOn = !bAliveLEDOn;

        // Report any captured data
        uint32_t uFirstIndex(0), uActualSamples(5);
        const uint8_t *pData(cADC.getNextSamples(uFirstIndex, uActualSamples));
        if (pData != nullptr) {
            for(uint32_t i=0; i<uActualSamples; i++) {
                printf("%d, %d\r\n", uFirstIndex+i, pData[i]);
            }
            continue;
        }

        // Process any command received
guKT5Line = __LINE__;
        _Command cCommand;
        if (true == _getCommand(cCommand)) {
            switch(cCommand.muOpCode) {
                case kInput: 
                    printf("Raw reading = %d\r\n", cMeasure.getRaw()); 
                    break;
                case kReportKts:
                    printf("Input/kts mappings:\r\n");
                    for(uint i=0; i<kKtsPositions; i++) {
                        printf("%s%dkts=%d", (0==i)?"":", ", i, scSettings.mpKtsForReading[i]);
                    }
                    printf("\r\n");
                    break;
                case kSetKtsVal:
                    if (cCommand.muData1 > 8) {
                        printf("Kts value %d out of range (0-8)\r\n", cCommand.muData1);
                        break;
                    }
                    scSettings.mpKtsForReading[cCommand.muData1] = cCommand.muData2;
                    printf("%dkts map to raw reading %d\r\n", cCommand.muData1, cCommand.muData2);
                    break;

                case kReportServoKts:
                    printf("Servo/kts mappings:\r\n");
                    for(uint i=0; i<kKtsPositions; i++) {
                        printf("%s%dkts=%d", (0==i)?"":", ", i, scSettings.mpServoKts[i]);
                    }
                    printf("\r\n");
                    break;
                case kSetServoKts:
                    if (cCommand.muData1 > 8) {
                        printf("Kts value %d out of range (0-8)\r\n", cCommand.muData1);
                        break;
                    }
                    scSettings.mpServoKts[cCommand.muData1] = cCommand.muData2;
                    printf("%dkts map to servo position %d\r\n", cCommand.muData1, cCommand.muData2);
                    break;
                case kServoRate: {
                    scSettings.muServoRate = cCommand.muData1;
                    printf("servo rate %d\r\n", scSettings.muServoRate); 
                    cDial.setRate(scSettings.muServoRate);
                    break;
                }
                case kServoOn: 
                    printf("servo AUTO\r\n"); 
                    bServoAuto = true; 
                    break;
                case kServoPos: {
                    uint8_t uPosn(cCommand.muData1);
                    bServoAuto = false;
                    printf("servo position = %d\r\n", uPosn);
                    float fPosn((float)uPosn / 255.0f);
                    cDial.setPosition(fPosn);
                    break;
                }
                case kServoTime: {
                    scSettings.muServoMin = cCommand.muData1;
                    scSettings.muServoMax = cCommand.muData2;
                    float fMinPulse(((float)scSettings.muServoMin * 10.0e-6f) + 300.0e-6f);
                    float fMaxPulse(((float)scSettings.muServoMax * 10.0e-6f) + 300.0e-6f);
                    printf("min = %.6f, max = %.6f\r\n", fMinPulse, fMaxPulse);
                    cDial.setPulseLengths(fMinPulse, fMaxPulse);
                    break;
                }
                case kSave: {
                    printf("Saving all current settings to flash\r\n");
                    bool bSuccess(cFlash.writeBlock(kKT5SettingsSignature, (const uint8_t*)&scSettings, sizeof(scSettings), _commandProcessor));
                    printf("\t%s\r\n", (true==bSuccess)?"Ok":"Failed");
                    break;
                }
                case kReportADC: {
                    uint32_t uSamples((uint32_t)roundf(((float)cCommand.muData1 * 1.0e-2f) * kSampleRateHz));
                    if (true == cADC.startCapture(uSamples)) {
                        printf("Capturing %d ADC samples - report when complete\r\n", uSamples);
                    } else {
                        printf("ADC capture of %d samples rejected.\r\n\ttoo large or capture in process\r\n", uSamples);
                    }
                    break;
                }
                case kResetFlash: {
                    if ((cCommand.muData1 == 91) && (cCommand.muData2 == 113)) {
                        printf("Erasing flash\r\n");
                        cFlash.eraseAndReset(_commandProcessor);
                        printf("\terase complete\r\n");
                    } else {
                        printf("Incorrect enable code entered.\r\n");
                    }
                    break;
                }
                default:
                    printf("Command %d, a1:%d, a2:%d\r\n", cCommand.muOpCode, cCommand.muData1, cCommand.muData2);
                    break;
            }
        }

        // Update the display
guKT5Line = __LINE__;
        char pBuffer[64];
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
                sprintf(pBuffer, "@(4,16,-6)%.3f@(2,20,34)avg KTS", fKtsAvg);
                break;
            }
            case kDisplayDistance: default: {
                float fNM(cMeasure.getDistanceTravelledNm());
                sprintf(pBuffer, "@(4,30,-6)%.2f@(2,20,34)SEA nm", fNM);
                break;
            }
        }

guKT5Line = __LINE__;
        std::string sDisplay(pBuffer);
guKT5Line = __LINE__;
        cDisplay.show(sDisplay);
guKT5Line = __LINE__;
    }
}
