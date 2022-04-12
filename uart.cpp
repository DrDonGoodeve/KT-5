/******************************************************************************
 * uart.cpp
 * 
 * Driver for RP pico UART(s). Implements a buffered send/receive model.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 10Apr2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/


// Includes
//*****************************************************************************
#include "uart.h"
#include <string.h>
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "utilities.h"


// Defines
//*****************************************************************************
#define kBufferSizeLimit    (1024)


// Local functions
//*****************************************************************************
static void _irqHandler0(void) {
    if (nullptr == UART::smpInstances[0]) {
        return;
    }
    UART::smpInstances[0]->irqHandler();
}

static void _irqHandler1(void) {
    if (nullptr == UART::smpInstances[1]) {
        return;
    }
    UART::smpInstances[1]->irqHandler();
}


// UART:: class
//*****************************************************************************
UART *UART::smpInstances[2] = {nullptr, nullptr};

UART::UART(uint uUart, uint uBaud, uint uData, uint uStop, uart_parity_t uParity) {
    // Indentify UART hardware resources
    mcUART = (0==uUART)?uart0:uart1;
    uint uTxPin((0==uUART)?0:?), uRxPin((0==uUART)?1:?);
    uint uIRQ((0==uUART) ? UART0_IRQ : UART1_IRQ);
    void (*pIRQHandler)(void) = (0==uUART)?_irqHandler0:_irqHandler1;

    // Set up buffering
    msReadBuffer.setSize(kBufferSizeLimit);

    // Set up UART peripheral
    uart_init(mcUART, uBaud);
    uart_set_hw_flow(mcUART, false, false);
    uart_set_format(mcUART, uData, uStop, uParity);
    uart_set_fifo_enabled(UART_ID, true);

    // Set the TX and RX pins by using the function select on the GPIO
    gpio_set_function(uTxPin, GPIO_FUNC_UART);
    gpio_set_function(uRxPin, GPIO_FUNC_UART);

    // Setup interrupt (Rx only)
    irq_set_exclusive_handler(uIRQ, pIRQHandler);
    irq_set_enabled(uIRQ, true);
    uart_set_irq_enables(uIRQ, true, false);    // Rx only
}

UART::~UART() {
    uint uIRQ((uart0==mcUART) ? UART0_IRQ : UART1_IRQ);
    irq_set_enabled(uIRQ, false);
    uart_deinit(mcUART);
}

uint UART::setBaud(uint uNewBaud) {
    return uart_set_baud(mcUART, uNewBaud);
}

bool UART::send(const std::string &sText) {
    return uart_puts(mcUART, sText.c_str());
}

void UART::irqHandler(void) {
    while (true == uart_is_readable(mcUART)) {
        uint8_t c(uart_getc(mcUART));
        if (muReadAvailable < kBufferSizeLimit) {
            mpReadBuffer[muReadAvailable++] = c;
        }
    }
}

uint UART::receive(std::string &sText, const std::string &sBreak) {
    ScopedLock cLock(mcReadBufferLock);
    mpBuffer[muReadAvailable] = '\0';
    if (false == sBreak.empty()) {
        sText = std::string(mpBuffer);
        uint uSize(muReadAvailable);
        muReadAvailable = 0;
        return uSize;

    } else {
        const char *pFind(strstr(mpBuffer, sBreak.c_str()));
        if (nullptr == pFind) {
            sText = "";
            return 0;
        }

        // Return the prefix of the break string, not including it
        uint uOffset(pFind - mpBuffer);
        uint uBreakLength(sBreak.size());
        uint uConsume(uOffset+uBreakLength);
        mpBuffer[uOffset] = '\0';
        sText = std::string(mpBuffer);
        memcpy(mpBuffer, &mpBuffer[uConsume], muReadAvailable-uConsume);
        muReadAvailable -= uConsume;
        return uOffset;
    }
}

