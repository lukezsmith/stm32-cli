#include "gpio_module.h"

const uint16_t ALL_PINS_OFF = 0;
const uint16_t ALL_PINS_ON = 0xffff;

uint16_t convertGPIOPinNumberToBit(int pinNumber)
{
    if (pinNumber <= 0 || pinNumber > 16)
        return 0;
    return 1 << (pinNumber -1 );
}

void GPIO_Set(uint32_t * gpioAddress, uint16_t pins)
{
    if (pins <= ALL_PINS_OFF || pins > ALL_PINS_ON)
        return;
    *gpioAddress |= pins;
}

void GPIO_Reset(uint32_t * gpioAddress,uint16_t pins)
{
    if (pins <= ALL_PINS_OFF || pins > ALL_PINS_ON)
        return;
    *gpioAddress &= ~(pins);
}

void GPIO_SetAll(uint32_t * gpioAddress)
{
    *gpioAddress = ALL_PINS_ON;
}

void GPIO_Toggle(volatile uint32_t * gpioAddress, uint16_t pins)
{
    if (pins <= ALL_PINS_OFF || pins > ALL_PINS_ON)
        return;
    *gpioAddress ^= pins;
}