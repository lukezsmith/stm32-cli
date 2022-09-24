#include "uart_module.h"
// #include "stm32f4xx.h"

// function to initialise UARTx
void UART_Init(volatile uint32_t *uartBaseAddress, int wordLength, float numStopBits, int baudRate)
{
    // Enable UART
    UART_Enable((uint32_t *)((uint8_t *)uartBaseAddress + UART_CR1_OFFSET));

    // Set word length
    UART_SetWordLength((uint32_t *)((uint8_t *)uartBaseAddress + UART_CR1_OFFSET), wordLength);

    // Set stop bits
    UART_SetStopBits((uint32_t *)((uint8_t *)uartBaseAddress + UART_CR2_OFFSET), numStopBits);

    // Set baud rate
    UART_SetBaudRate((uint32_t *)((uint8_t *)uartBaseAddress + UART_BRR_OFFSET), baudRate);
}

void UART_EnableRX(volatile uint32_t *uartBaseAddress)
{
    *uartBaseAddress |= (1 << 2);
}

void UART_EnableTX(volatile uint32_t *uartBaseAddress)
{
    *uartBaseAddress |= (1 << 3);
}

void UART_EnableRXNEIE(volatile uint32_t *uartBaseAddress)
{
    *uartBaseAddress |= (1 << 5);
}

void UART_EnableTXEIE(volatile uint32_t *uartBaseAddress)
{
    *uartBaseAddress |= (1 << 7);
}



static uint16_t convertGPIOPinNumberToBit(int pinNumber)
{
    return 1 << (pinNumber - 1);
}
void UART_Enable(volatile uint32_t *uartBaseAddress)
{
    *uartBaseAddress |= (1 << 13);
}

void UART_SetWordLength(volatile uint32_t *uartBaseAddress, int length)
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

void UART_SetStopBits(volatile uint32_t *uartBaseAddress, float length)
{
    if (length != 1.0 && length != 0.5 && length != 2.0 && length != 1.5)
        return;

    *uartBaseAddress &= ~(UART_STOP_BITS_1_5 << (11 * 2));

    if (length == 1)
        *uartBaseAddress |= UART_STOP_BITS_1 << 11;
    if (length == 0.5)
        *uartBaseAddress |= UART_STOP_BITS_0_5 << 11;
    if (length == 2.0)
        *uartBaseAddress |= UART_STOP_BITS_2 << 11;
    if (length == 1.5)
        *uartBaseAddress |= UART_STOP_BITS_1_5 << 11;
}

void UART_SetBaudRate(volatile uint32_t *uartBaseAddress, int baudRate)
{
    // clear bits
    *uartBaseAddress &= ~0xffff;

    // set mantissa and fraction BDD bits
    *uartBaseAddress = (UART_BDD_FRACTION_LOW_SPEED << 0) | (UART_BDD_MANTISSA_LOW_SPEED << 4);
}
void UART_Transmit(volatile uint32_t *uartBaseAddress, char c)
{
    while(!(*((uint32_t *)((uint8_t *)uartBaseAddress + UART_SR_OFFSET))& (1 << 6))){
    }
    *((uint32_t *)((uint8_t *)uartBaseAddress + UART_DR_OFFSET)) = c;
}

char UART_Receive(volatile uint32_t *uartBaseAddress)
{
    while (!(*(uint32_t *)((uint8_t *)uartBaseAddress + UART_SR_OFFSET)& (1 << 5)));
    char c = *(uint32_t *)((uint8_t *)uartBaseAddress + UART_DR_OFFSET);
    return c;

}

void UART_TransmitString(volatile uint32_t *uartBaseAddress, char *p)
{
    while (*p != '\0')
    {
        UART_Transmit(uartBaseAddress, *p);
        p++;
    }
}