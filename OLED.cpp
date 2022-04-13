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
#include <stdio.h>
#include "OLED.h"
#include "pico-ssd1306/ssd1306.h"
#include "pico-ssd1306/textRenderer/TextRenderer.h"
#include "hardware/i2c.h"
using namespace pico_ssd1306;


// Includes
//*****************************************************************************
#define kI2C_SDA    (12)
#define kI2C_SCL    (13)
#define kI2CFreq    (1000000)


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
static uint _renderChar(SSD1306 *pDevice, const Character &cChar, uint uX, uint uY) {
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
                pDevice->setPixel(uX+uCol, uY+uRow-6);
            }
            uByte = (uByte>>1);
            uBitsRemaining--;
        }
    }
    return cChar.uWidth;
}

#define _bigToLittleEndian16(a)     ((((a)&0xff)<<8) | (((a)&0xff00)>>8))

static void _renderVWText(SSD1306 *pDevice, const char *pText, const uint8_t *pVWFont, uint uX, uint uY) {
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
            uX += cChar.uWidth;
            continue;
        }

        uX += _renderChar(pDevice, cChar, uX, uY);
        printf("uChar:0x%02x, pJump->uOffset=%d, cChar.uBytes=%d, cChar.uWidth=%d, cChar.uHeight=%d\r\n", uChar, uOffsetLE16, cChar.uBytes, cChar.uWidth, cChar.uHeight);
    }
}


// OLED:: 
/// Class method implementations
//*****************************************************************************
// Constructor - connect to OLED device and prepare to render
OLED::OLED(void) : 
    mpDisplay(nullptr) {

    // Init i2c0 controller
    i2c_init(i2c0, kI2CFreq);

    // Set up GPIO12 (pin 16), GPIO13 (pin 17)
    gpio_set_function(kI2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(kI2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(kI2C_SDA);
    gpio_pull_up(kI2C_SCL);

    // Pause for SSD1306 to self-initialize (assuming called soon after boot)
    sleep_ms(200);

    // Create a new display object at address 0x3C and size of 128x32
    mpDisplay = new SSD1306(i2c0, 0x3C, Size::W128xH32);

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
extern const char pFontLatoMedium26[];
extern const char pFontLatoBlack32[];
extern const char pFontLatoBlack38[];

void OLED::show(const char *pString) {
    mpDisplay->clear();
    _renderVWText(mpDisplay, pString, (const uint8_t *)pFontLatoBlack38, 0 ,0);
    mpDisplay->sendBuffer();
}

void OLED::clear(void) {
    printf("OLED clear\r\n");
}

void OLED::setContrast(uint8_t uContrast) {
    mpDisplay->setContrast(uContrast);
}


void OLED::scrollOut(bool bLeftNotRight) {
    printf((true == bLeftNotRight)?"Scroll out left\r\n":"Scroll out right\r\n");
}
