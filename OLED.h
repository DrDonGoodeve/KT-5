/******************************************************************************
 * OLED.h
 * 
 * Display driving for OLED 128x32 display. Code heavily borrowed from
 * pico example code for OLED driver over I2C.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 1Apr2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

#ifndef _OLED_
#define _OLED_


// Includes
//-----------------------------------------------------------------------------
#include "pico/stdlib.h"
#include "pico-ssd1306/ssd1306.h"


// SpeedMeasurement class
/// Primary function is to estimate speed from the transducer signal. Creates
/// derived measurements.
//-----------------------------------------------------------------------------
class OLED {
    private:
        pico_ssd1306::SSD1306 *mpDisplay;

    public:
        /// Initialize the class and set accumulated time to zero.
        OLED(void); // Connect with OLED device

        /// Destructor 
        ~OLED();

        /// Display mechanisms. pString is a conventional string with
        /// embedded control codes. Default is start at top left (0,0)
        /// after clear.
        /// @l - large font (16x32px)
        /// @r - regular font (12x16px)
        /// @s - small font (8x10px)
        /// @h<xx> - horizontal start in px (0-127)
        /// @v<xx> - vertical start in px (0-31)
        void show(const char *pString);

        // Clear and reset display
        void clear(void);

        // Set display contrast
        void setContrast(uint8_t uContrast);

        // Scroll out the entire display to left or right until cleared
        void scrollOut(bool bLeftNotRight);
};

#endif // _OLED_