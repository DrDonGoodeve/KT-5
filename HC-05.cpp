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


// Includes
//*****************************************************************************
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "utilities.h"
#include "HC-05.h"


// Defines
//*****************************************************************************
#define kUARTInst       (uart0)     // Pin 1:Tx, 2:Rx
#define kUARTTx         (0)        // Pin 1
#define kUARTRx         (1)        // Pin 2
#define kDataBaud       (9600)
#define kATBaud         (36400)
#define kDataBits       (8)
#define kStopBits       (1)
#define kParity         (UART_PARITY_NONE)
#define kATKeyGPIO      (2)         // pin 4 - AT is high. Driven low.
#define kRxBufferBytes  (1024)


// Local types
//*****************************************************************************
// Buffer object - holds a sequence of bytes and potentially chains to further
// objects of the same type to form a longer sequence.
//-----------------------------------------------------------------------------
class _Buffer {
    private:
        uint8_t mpBuffer[kRxBufferBytes];
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
    uint uCharsRxd(0);
    while (true == uart_is_readable(kUARTInst)) {
        uint8_t c(uart_getc(kUARTInst));
        // Can we send it back?
        if (true == uart_is_writable(kUARTInst)) {
            // Change it slightly first!
            c++;
            uart_putc(kUARTInst, c);
        }
        uCharsRxd++;
    }
}


// HC05::
//*****************************************************************************
HC05::HC05(const char *pID) {
    // Setup AT control pin and de-assert
    gpio_init(kATKeyGPIO);
    gpio_set_dir(kATKeyGPIO, GPIO_OUT);
    gpio_put(kATKeyGPIO, 0);

    // Set up our UART with a basic baud rate.
    uart_init(kUARTInst, kDataBaud);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(kUARTTx, GPIO_FUNC_UART);
    gpio_set_function(kUARTRx, GPIO_FUNC_UART);

    // Set UART flow control, data bits and parity
    //uart_set_hw_flow(kUARTInst, false, false);

    // Set our data format
    uart_set_format(kUARTInst, kDataBits, kStopBits, kParity);

    // Enable UART Fifo
    //uart_set_fifo_enabled(kUARTInst, true);

/*
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
*/
}

HC05::~HC05() {

}

uint HC05::read(char *pBuffer, uint uBufferSize) {
    return 0;
}

bool HC05::write(const char *pBuffer) {
    return false;
}

