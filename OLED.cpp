/******************************************************************************
 * OLED.cpp
 * 
 * Display driving for OLED 128x64 display. Code heavily borrowed from
 * pico example code for OLED driver over I2C.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 1Apr2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

// Includes
//*****************************************************************************
#include "OLED.h"
#include <stdio.h>


// Local functions
//*****************************************************************************


// OLED:: 
/// Class method implementations
//*****************************************************************************
// Constructor - connect to OLED device and prepare to render
OLED::OLED(void) {
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
