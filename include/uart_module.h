#include <stdint.h>

// #define uint32_t UART_STOP_BITS_1    0x0;
// #define uint32_t UART_STOP_BITS_0_5  0x1;
// #define uint32_t UART_STOP_BITS_2    0x2;
// #define uint32_t 

void UART_Enable(uint32_t * uartAddress);
void UART_SetWordLength(uint32_t * uartBaseAddress, int length);
void UART_SetStopBits(uint32_t * uartBaseAddress, double length);
void UART_SetBaudRate(uint32_t *uartBaseAddress, int baudRate);
void UART_Transmit(uint32_t *uartBaseAddress, uint8_t message);
uint32_t UART_Receive(uint32_t *uartBaseAddress);