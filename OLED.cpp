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

// Includes
//*****************************************************************************
#define kI2C_SDA    (12)
#define kI2C_SCL    (13)
#define kI2CFreq    (1000000)


// Local functions
//*****************************************************************************


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
    using namespace pico_ssd1306;
    mpDisplay = new SSD1306(i2c0, 0x3C, Size::W128xH32);

    // Here we rotate the display by 180 degrees, so that it's not upside down from my perspective
    // If your screen is upside down try setting it to 1 or 0
    mpDisplay->setOrientation(0);

    // Send buffer to the display
    mpDisplay->sendBuffer();
}

/// Destructor 
OLED::~OLED() {
    delete mpDisplay;
}

/// Display mechanisms
void OLED::show(const char *pString) {
    mpDisplay->clear();
    drawText(mpDisplay, font_16x32, pString, 0 ,0);

    for(uint i=0; i<32; i++) {
        mpDisplay->setPixel(0, i);
        mpDisplay->setPixel(127,i);
    }
    for(uint i=0; i<128; i++) {
        mpDisplay->setPixel(i, 0);
        mpDisplay->setPixel(i, 31);
    }
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
