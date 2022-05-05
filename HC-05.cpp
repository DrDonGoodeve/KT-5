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
#include "pico/stdio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "utilities.h"
#include "HC-05.h"


// Defines
//*****************************************************************************
#define kUARTInst       (uart0)     // Pin 1:Tx, 2:Rx
#define kUARTTx         (0)        // Pin 1
#define kUARTRx         (1)        // Pin 2
//#define kUARTTx         (16)        // Pin 21
//#define kUARTRx         (17)        // Pin 22
#define kDataBaud       (9600)
#define kATBaud         (38400)
#define kDataBits       (8)
#define kStopBits       (1)
#define kParity         (UART_PARITY_NONE)
#define kATKeyGPIO      (2)         // pin 4 - AT is high. Default (data) is low.


// Local types
//*****************************************************************************

// Local functions
//*****************************************************************************

// HC05::
//*****************************************************************************
HC05::HC05(void) {
    // Setup AT control pin and de-assert
    gpio_init(kATKeyGPIO);
    gpio_set_dir(kATKeyGPIO, GPIO_OUT);
    gpio_put(kATKeyGPIO, 0);    // Set into data mode

    // Set the TX and RX pins by using the function select on the GPIO
    // and initialize UART for AT operation
    gpio_set_function(kUARTTx, GPIO_FUNC_UART);
    gpio_set_function(kUARTRx, GPIO_FUNC_UART);
    uart_init(kUARTInst, kDataBaud); 
    uart_set_hw_flow(kUARTInst, false, false);// Set UART flow control, data bits and parity
    uart_set_format(kUARTInst, kDataBits, kStopBits, kParity); // Set our data format
}

HC05::~HC05() {
}


