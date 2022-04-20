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
#include <string>
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

        /// Display mechanisms. sString may contain embedded
        /// control information using a tag format of
        /// '@(<tag>)' where <tag> is one of:
        /// <font>, <font><iX><iY>
        void show(const std::string &sText);

        // Clear and reset display
        void clear(void);

        // Set display contrast
        void setContrast(uint8_t uContrast);

        // Scroll out the entire display to left or right until cleared
        void scrollOut(bool bLeftNotRight);
};

#endif // _OLED_