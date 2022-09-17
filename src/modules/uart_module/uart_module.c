#include "uart_module.h"

static uint16_t convertGPIOPinNumberToBit(int pinNumber)
{
    return 1 << (pinNumber -1 );
}
void UART_Enable(uint32_t * uartBaseAddress)
{
    *uartBaseAddress |= 0x1000;
}

void UART_SetWordLength(uint32_t * uartBaseAddress, int length)
{
    if (length != 8 && length != 9)
        return;
    if (length == 8){
        *uartBaseAddress |= 0;
    }else{
    *uartBaseAddress |= 0x800;
    }
}