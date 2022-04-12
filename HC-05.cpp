/******************************************************************************
 * HC-05.h
 * 
 * Driver for HC-05 bluetooth module serial data connection. Heavily based
 * on UART code examples from Raspberry Pi examples.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 8Apr2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

#ifndef _ADCENGINE_
#define _ADCENGINE_

// Includes
//*****************************************************************************
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/uart.h"
#include "hardware/irq.h"


// Defines
//*****************************************************************************
#define kUARTInst       (uart0)     // Pin 1:Tx, 2:Rx
#define kUARTTx         (0)
#define kUARTRx         (1)
#define kDataBaud       (9600)
#define kATBaud         (36400)
#define kDataBits       (8)
#define kStopBits       (1)
#define kParity         (UART_PARITY_NONE)
#define kATKeyGPIO      (2)         // pin 4 - AT is high. Pulled down.
#define kRxBufferBytes  (1024)


// Local types
//*****************************************************************************
// Simple scoped object for holding and releasing a critical section as
// required.
//-----------------------------------------------------------------------------
class ScopedLock {
    private:
        critical_section_t *mpCriticalSection;
        
    public:
        ScopedLock(critical_section_t *pCriticalSection) :
            mpCriticalSection(pCriticalSection) {
            critical_section_enter_blocking(mpCriticalSection);
        }

        ~ScopedLock() {
            critical_section_exit(mpCriticalSection);
        }
};

// Buffer object - holds a sequence of bytes and potentially chains to further
// objects of the same type to form a longer sequence.
//-----------------------------------------------------------------------------
class _Buffer {
    private:
        uint8_t mpBuffer[kBufferBytes];
        uint muUsed;
        uint muRead;
        _Buffer *mpNextBuffer;

    public:
        _Buffer(void);
        ~_Buffer();

        bool write(uint8_t c) {
            uint uWritable(uart_is_writable(kUARTInst));
            if (uWritable > 0) {
                uart_putc(kUARTInst, c);
                return true;
            } 
            return false;
        }

        uint write(uint8_t *pChars, uint uChars) {
            uint uWritable(uart_is_writable(kUARTInst));
            uint uWriteRemaining((uChars>uWritable)?uWritable:uChars);
            uint uWritten(0);
            while(uWriteRemaining>0) {
                uart_putc(kUARTInst, *(pChars++));
                uWriteRemaining--;
                uWritten++;
            }
            return uWritten;          
        }

        bool readChar(uint8_t &c);
};


// Local functions
//*****************************************************************************

// RX interrupt handler
static void _uartRxIRQ(void) {
    while (true == uart_is_readable(kUARTID)) {
        uint8_t c(uart_getc(kUARTID));
        // Can we send it back?
        if (uart_is_writable()) {
            // Change it slightly first!
            ch++;
            uart_putc(UART_ID, ch);
        }
        chars_rxed++;
    }
}

int main() {
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, 2400);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    // OK, all set up.
    // Lets send a basic string out, and then run a loop and wait for RX interrupts
    // The handler will count them, but also reflect the incoming data back with a slight change!
    uart_puts(UART_ID, "\nHello, uart interrupts\n");

    while (1)
        tight_loop_contents();
}

/// \end:uart_advanced[]
