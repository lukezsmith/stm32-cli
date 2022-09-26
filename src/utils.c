#include "utils.h"
#include "uart_module.h"
#include "stm32f407xx.h"

char getCommandCode(char *buffer)
{
    // return buffer - 48;  // ASCI char to uint8_t
    return buffer[0] - 48; // ASCI char to uint8_t
}

void printErrorMessage(void)
{
    UART_TransmitString((volatile uint32_t *) USART2,"Invalid command selection. Enter a valid command option.");

}
