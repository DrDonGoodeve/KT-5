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
#define kI2CPin0    (12)
#define kI2CPin1    (13)
#define kI2CFreq    (1000000)


// Local functions
//*****************************************************************************


// OLED:: 
/// Class method implementations
//*****************************************************************************
// Constructor - connect to OLED device and prepare to render
OLED::OLED(void) {
    // Init i2c0 controller
    i2c_init(i2c0, kI2CFreq);

    // Set up pins 12 and 13
    gpio_set_function(kI2CPin0, GPIO_FUNC_I2C);
    gpio_set_function(kI2CPin1, GPIO_FUNC_I2C);
    gpio_pull_up(kI2CPin0);
    gpio_pull_up(kI2CPin1);

    // Pause for SSD1306 to self-initialize (assuming called soon after boot)
    sleep_ms(250);

    // Create a new display object at address 0x3D and size of 128x64
    using namespace pico_ssd1306;
    SSD1306 display = SSD1306(i2c0, 0x3D, Size::W128xH32);

    // Here we rotate the display by 180 degrees, so that it's not upside down from my perspective
    // If your screen is upside down try setting it to 1 or 0
    display.setOrientation(0);

    // Draw text on display
    // After passing a pointer to display, we need to tell the function what font and text to use
    // Available fonts are listed in textRenderer's readme
    // Last we tell this function where to anchor the text
    // Anchor means top left of what we draw
    // We use \x escape to use chars by their hex numeration according to the ASCII table
    drawText(&display, font_5x8, "\x24 \xba \xb2", 0 ,0);
    drawText(&display, font_8x8, "\x24 \xba \xb2", 0 ,10);
    drawText(&display, font_12x16, "\x24 \xba \xb2", 0 ,20);
    drawText(&display, font_16x32, "\x24 \xba \xb2", 0 ,36);

    // Send buffer to the display
    display.sendBuffer();
}

/// Destructor 
OLED::~OLED() {
}

/// Display mechanisms
void OLED::show(const char *pString) {
    printf("%s/r/n", pString);
}

void OLED::clear(void) {
    printf("OLED clear\r\n");
}

void OLED::scrollOut(bool bLeftNotRight) {
    printf((true == bLeftNotRight)?"Scroll out left\r\n":"Scroll out right\r\n");
}
