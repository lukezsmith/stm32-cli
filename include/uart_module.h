#include <stdint.h>

// Stop bit values
#define UART_STOP_BITS_1       (uint32_t) 0
#define UART_STOP_BITS_0_5     (uint32_t) 1
#define UART_STOP_BITS_2       (uint32_t) 2
#define UART_STOP_BITS_1_5     (uint32_t) 3

// Baud rate constants
#define UART_CLOCK_SPEED   (uint32_t) 42000000

// USARTDIV = 72MHz / (16 * Baud Rate)
// Ex: 115200
// 72MHz / (16 * 115200) = 22.786
// Mantissa = 22
// Fractional = 0.786 * 16 = 12.576 = 13
#define UART_BDD_FRACTION_LOW_SPEED  (uint32_t) 11
#define UART_BDD_MANTISSA_LOW_SPEED  (uint32_t) 8
#define UART_BDD_FRACTION_VERY_HIGH_SPEED  (uint32_t) 5
#define UART_BDD_MANTISSA_VERY_HIGH_SPEED  (uint32_t) 195

// USART CR1 Register offset
#ifndef TEST
#define UART_CR1_OFFSET (uint32_t) 0x0C
#define UART_CR2_OFFSET (uint32_t) 0x10
#define UART_BRR_OFFSET (uint32_t) 0x08
#define UART_DR_OFFSET  (uint32_t) 0x04
#define UART_SR_OFFSET  (uint32_t) 0x00
#else
extern uint32_t UART_DR_OFFSET;
extern uint32_t UART_SR_OFFSET;
#endif

void UART_Init(volatile uint32_t *uartBaseAddress, int wordLength, float numStopBits, int baudRate);
void UART_Enable(volatile uint32_t * uartAddress);
void UART_EnableRX(volatile uint32_t *uartBaseAddress);
void UART_EnableTX(volatile uint32_t *uartBaseAddress);
void UART_SetWordLength(volatile uint32_t * uartBaseAddress, int length);
void UART_SetStopBits(volatile uint32_t * uartBaseAddress, float length);
void UART_SetBaudRate(volatile uint32_t *uartBaseAddress, int baudRate);
void UART_Transmit(volatile uint32_t *uartBaseAddress, char c);
void UART_TransmitString(volatile uint32_t *uartBaseAddress, char *p);
char UART_Receive(volatile uint32_t *uartDR);