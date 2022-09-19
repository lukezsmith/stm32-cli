#include "uart_module.h"

// Stop bit values
const uint32_t UART_STOP_BITS_1 =       0;
const uint32_t UART_STOP_BITS_0_5 =     1;
const uint32_t UART_STOP_BITS_2 =       2;
const uint32_t UART_STOP_BITS_1_5 =     3;

// Baud rate constants
static const uint32_t UART_CLOCK_SPEED = 45000000;
static const uint32_t UART_BDD_FRACTION = 7;
static const uint32_t UART_BDD_MANTISSA = 24;


static uint16_t convertGPIOPinNumberToBit(int pinNumber)
{
    return 1 << (pinNumber - 1);
}
void UART_Enable(uint32_t *uartBaseAddress)
{
    *uartBaseAddress |= 0x1000;
}

void UART_SetWordLength(uint32_t *uartBaseAddress, int length)
{
    if (length != 8 && length != 9)
        return;
    if (length == 8)
    {
        *uartBaseAddress |= 0;
    }
    else
    {
        *uartBaseAddress |= 0x800;
    }
}

void UART_SetStopBits(uint32_t *uartBaseAddress, double length)
{   
    if (length != 1.0 && length != 0.5 && length != 2.0 
        && length != 1.5 )
        return;

    *uartBaseAddress &= ~(UART_STOP_BITS_1_5 << (11 * 2));

    if (length  == 1)
        *uartBaseAddress |= UART_STOP_BITS_1 << 11;
    if (length  == 0.5)
        *uartBaseAddress |= UART_STOP_BITS_0_5 << 11;
    if (length  == 2.0)
        *uartBaseAddress |= UART_STOP_BITS_2 << 11;
    if (length  == 1.5)
        *uartBaseAddress |= UART_STOP_BITS_1_5 << 11;
}

void UART_SetBaudRate(uint32_t *uartBaseAddress, int baudRate)
{
    // clear bits
    *uartBaseAddress &= ~0xffff;

    // set mantissa and fraction BDD bits
    *uartBaseAddress = (UART_BDD_FRACTION << 0) | (UART_BDD_MANTISSA << 4);

}
void UART_Transmit(uint32_t *uartBaseAddress, uint8_t message)
{
    *uartBaseAddress = message;    
}

uint32_t UART_Receive(uint32_t *uartBaseAddress)
{
    return *uartBaseAddress;
}