/******************************************************************************
 * uart.h
 * 
 * Driver for RP pico UART(s). Implements a buffered send/receive model.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 10Apr2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

#ifndef _UART_
#define _UART_


// Includes
//*****************************************************************************
#include <string>
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "utilities.h"


// Defines
//*****************************************************************************
#define kBufferSizeLimit    (1024-1)


// UART:: class
//*****************************************************************************
class UART {
    private:
        friend static void _irqHandler0(void);
        friend static void _irqHandler1(void);

        static UART *smpInstances[2];   // Used by IRQ to obtain class info
        uart_inst_t mcUART;

        critical_section_t mcReadBufferLock;
        uint muReadAvailable;
        char mpBuffer[kBufferSizeLimit+1];
        void irqHandler(void);

    public:
        UART(uint uUart, uint uBaud=9600, uint uData=8, uint uStop=1, uart_parity_t uParity=UART_PARITY_NONE);
        ~UART();

        uint setBaud(uint uNewBaud);

        bool send(const std::string &sText);
        uint receive(std::string &sText, const std::string &sBreak="");
};

#endif // _UART_
