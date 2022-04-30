/******************************************************************************
 * OLED.cpp
 * 
 * Display driving for OLED 128x32 display. Code relies on pico-ssd1306 from
 * https://github.com/Harbys/pico-ssd1306. Looks to be well designed and
 * clean code. Many thanks. Note this code is distributed under BSD v3
 * license - which is reproduced in the source code for that library.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 1Apr2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

// Includes
//*****************************************************************************
#include <cstdlib>
#include <stdio.h>
#include <string>
#include <list>
#include "OLED.h"
#include "pico-ssd1306/ssd1306.h"
#include "pico-ssd1306/textRenderer/TextRenderer.h"
#include "hardware/i2c.h"
using namespace pico_ssd1306;
#include "utilities.h"


// Includes
//*****************************************************************************
//#define kI2C_SDA    (12)
//#define kI2C_SCL    (13)
#define kI2C_SDA    (18)
#define kI2C_SCL    (19)
#define kI2CFreq    (1000000)

volatile uint guOLEDLine = 0;

// Local types
//*****************************************************************************
// Extract from font data structure describing a single character
typedef struct {
    uint uBytes;
    const uint8_t *pData;
    uint uWidth;
    uint uHeight;
}Character;

// Font header interpretation
typedef struct {
    uint8_t uWidth;
    uint8_t uHeight;
    uint8_t uFirstChar;
    uint8_t uNumChars;
}FontHeader;

// Jump table entry interpretation
typedef struct {
    uint16_t uOffsetBE16;
    uint8_t uBytes;
    uint8_t uWidth;
}JumpEntry;


// Local functions
//*****************************************************************************
static uint _renderChar(SSD1306 *pDevice, const Character &cChar, int iX, int iY) {
    const uint8_t *pCharData(cChar.pData);
    uint8_t uByte(0x0);
    uint uBytesRemaining(cChar.uBytes);
    for(uint uCol=0; uCol<cChar.uWidth; uCol++) {
        uint uBitsRemaining(0);
        for(uint uRow=0; uRow<cChar.uHeight; uRow++) {
            if (0 == uBitsRemaining) {
                if (0 == uBytesRemaining) {
                    return cChar.uWidth;    // Data exhausted - we are done
                }
                uBytesRemaining--;
                uByte = *(pCharData++);
                uBitsRemaining = 8;
            }            
            if ((uByte & 0x1) != 0x0) {
                pDevice->setPixel(iX+uCol, iY+uRow);
                //pDevice->setPixel(iX+uCol, iY+(2*uRow));
                //pDevice->setPixel(iX+uCol, iY+(2*uRow)+1);
            }
            uByte = (uByte>>1);
            uBitsRemaining--;
        }
    }
    return cChar.uWidth;
}

#define _bigToLittleEndian16(a)     ((((a)&0xff)<<8) | (((a)&0xff00)>>8))

static void _renderVWText(SSD1306 *pDevice, const char *pText, const uint8_t *pVWFont, int &iX, int &iY) {
    if (nullptr == pVWFont) {
        return;
    }

    const FontHeader *pHeader(reinterpret_cast<const FontHeader*>(pVWFont));
    uint uFirstChar(pHeader->uFirstChar), uNumChars(pHeader->uNumChars);
    const uint8_t *pJumpBase(pVWFont + sizeof(FontHeader));
    const uint8_t *pCharBase(pJumpBase + (uNumChars * sizeof(JumpEntry)));

    const char *pChar(pText);
    while (*pChar != '\0') {
        uint8_t uChar(*(pChar++));
        if ((uChar < pHeader->uFirstChar) || (uChar > (uFirstChar + uNumChars))) {
            uChar = '?';    // Unprintable characters flagged by '?' which is assumed to be part of font
        }

        const JumpEntry *pJump(reinterpret_cast<const JumpEntry*>(pVWFont + sizeof(FontHeader) + ((uChar-pHeader->uFirstChar) * sizeof(JumpEntry))));
        uint16_t uOffsetLE16(_bigToLittleEndian16(pJump->uOffsetBE16));

        Character cChar;
        cChar.pData = pCharBase + uOffsetLE16;
        cChar.uBytes = pJump->uBytes;
        cChar.uWidth = pJump->uWidth;
        cChar.uHeight = pHeader->uHeight;
        if (uOffsetLE16 == 0xffff) {
            iX += cChar.uWidth;
            continue;
        }

        iX += _renderChar(pDevice, cChar, iX, iY);
        //printf("uChar:0x%02x, pJump->uOffset=%d, cChar.uBytes=%d, cChar.uWidth=%d, cChar.uHeight=%d\r\n", uChar, uOffsetLE16, cChar.uBytes, cChar.uWidth, cChar.uHeight);
    }
}


// OLED:: 
/// Class method implementations
//*****************************************************************************
// Constructor - connect to OLED device and prepare to render
OLED::OLED(void) : 
    mpDisplay(nullptr) {

    // Init i2c0 controller
    //i2c_init(i2c0, kI2CFreq);
    i2c_init(i2c1, kI2CFreq);

    // Set up GPIO12 (pin 16), GPIO13 (pin 17)
    gpio_set_function(kI2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(kI2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(kI2C_SDA);
    gpio_pull_up(kI2C_SCL);

    // Pause for SSD1306 to self-initialize (assuming called soon after boot)
    sleep_ms(200);

    // Create a new display object at address 0x3C and size of 128x32
    //mpDisplay = new SSD1306(i2c0, 0x3C, Size::W128xH32);
    //mpDisplay = new SSD1306(i2c0, 0x3C, Size::W128xH64);
    mpDisplay = new SSD1306(i2c1, 0x3C, Size::W128xH64);

    // Here we rotate the display by 180 degrees, so that it's not upside down from my perspective
    // If your screen is upside down try setting it to 1 or 0
    mpDisplay->setOrientation(0);

    // Send buffer to the display (clearing)
    mpDisplay->sendBuffer();
}

/// Destructor 
OLED::~OLED() {
    delete mpDisplay;
}

/// Display mechanisms
#define kInvalidCoord   (-1000.0f)
class _TextElement {
    public:
        const char *mpFont;
        std::string msText;
        int miX;
        int miY;

        _TextElement(const char *pFont, const std::string sText, int iX=kInvalidCoord, int iY=kInvalidCoord) :
            mpFont(pFont), msText(sText), miX(iX), miY(iY) {
        }
};

#include "VWFonts.h"
static void _split(const std::string &sTag, std::string &sFont, std::string &sX, std::string &sY) {
    sFont = sX = sY = "";
    size_t uFind(sTag.find(","));
    if (std::string::npos == uFind) {
        sFont = sTag;
    } else {    // Found first comma
        sFont = sTag.substr(0, uFind);
        std::string sRemainder(sTag.substr(uFind+1));
        uFind = sRemainder.find(",");
        if (std::string::npos == uFind) {
            sX = sRemainder;
        } else {    // Found second comma
            sX = sRemainder.substr(0, uFind);
            sY = sRemainder.substr(uFind+1);
        }
    }
}

void OLED::show(const std::string &sText) {
    static const char *pFonts[] = {
        pFontLatoMed13, pFontLatoMed13BoldItalic, pFontLatoMed26, pFontLatoBlack32, pFontLatoBlack38
    };
    const char *pFont(pFonts[0]);

    std::list<_TextElement> lPlotList;
    std::string sRemaining(sText);
    int iX(kInvalidCoord), iY(kInvalidCoord);
    guOLEDLine = __LINE__;
    while(false == sRemaining.empty()) {
        guOLEDLine = __LINE__;
        size_t uStart(sRemaining.find("@("));
        if (std::string::npos == uStart) {
            lPlotList.push_back(_TextElement(pFont, sRemaining, iX, iY));
            iX = iY = kInvalidCoord;
            sRemaining.clear();  
            guOLEDLine = __LINE__;

        } else if (uStart != 0) {
            std::string sText(sRemaining.substr(0,uStart));
            lPlotList.push_back(_TextElement(pFont, sText, iX, iY));
            iX = iY = kInvalidCoord;
            sRemaining = sRemaining.substr(uStart);
            guOLEDLine = __LINE__;

        } else {
            // Extract the tag
            size_t uTagEnd(sRemaining.find(")", uStart+2));
            if (std::string::npos == uTagEnd) {
                printf("Badly formed tag at %s\r\n", sRemaining.c_str());
                sRemaining.clear();
                guOLEDLine = __LINE__;

            } else {   
                std::string sTag(sRemaining.substr(uStart+2, uTagEnd-(uStart+2)));
                sRemaining = sRemaining.substr(uTagEnd+1);
                guOLEDLine = __LINE__;

                // Interpret tag (font,x,y)
                std::string sFont, sX, sY;
                _split(sTag, sFont, sX, sY);
                if (false == sFont.empty()) {
                    int iFont(atoi(sFont.c_str()));
                    iFont = ((iFont<0)?0:((iFont>=_arraysize(pFonts))?(_arraysize(pFonts)-1):iFont));
                    pFont = pFonts[iFont];
                }
                if (false == sX.empty()) {
                    iX = atoi(sX.c_str());
                }
                if (false == sY.empty()) {
                    iY = atoi(sY.c_str());
                }
                guOLEDLine = __LINE__;
            }
        }
    }
    guOLEDLine = __LINE__;

    mpDisplay->clear();
    int iPlotX(0), iPlotY(0);
        guOLEDLine = __LINE__;
    for(std::list<_TextElement>::const_iterator cIter = lPlotList.begin(); cIter != lPlotList.end(); ++cIter) {
         guOLEDLine = __LINE__;
       const _TextElement &cE(*cIter);
        iPlotX = (cE.miX != kInvalidCoord)?cE.miX:iX;
        iPlotY = (cE.miY != kInvalidCoord)?cE.miY:iY;
        guOLEDLine = __LINE__;
        _renderVWText(mpDisplay, cE.msText.c_str(), (const uint8_t *)cE.mpFont, iPlotX, iPlotY);
         guOLEDLine = __LINE__;
   }
        guOLEDLine = __LINE__;
    mpDisplay->sendBuffer();
        guOLEDLine = __LINE__;
}

void OLED::setContrast(uint8_t uContrast) {
    mpDisplay->setContrast(uContrast);
}
