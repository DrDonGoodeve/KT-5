/******************************************************************************
 * HC-05.h
 * 
 * Driver for HC-05 bluetooth module serial data connection. Heavily based
 * on UART code from Raspberry Pi examples.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 8Apr2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

#ifndef _HC05_
#define _HC05_

// Includes
//*****************************************************************************
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "utilities.h"


// Defines
//*****************************************************************************


// Class definitions
//*****************************************************************************
class HC05 {
    private:

    public:
        HC05(void);
        ~HC05();
};


#endif // _HC05_
