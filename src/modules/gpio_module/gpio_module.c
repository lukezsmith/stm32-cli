#include "gpio_module.h"

// convert specified pin to bits
uint16_t convertGPIOPinNumberToBit(int pinNumber)
{
    if (pinNumber <= 0 || pinNumber > 16)
        return 0;
    return 1 << (pinNumber -1 );
}

// function to initialise GPIO pin(s)
void GPIO_Init(volatile uint32_t *gpioBaseAddress, int pinNumber, uint32_t mode, uint32_t speed )
{
    // Set Port Mode
    GPIO_SetPortMode(gpioBaseAddress + GPIO_MODER_OFFSET, mode, pinNumber);

    // Set Port Speed
    // GPIO_SetPortSpeed(gpioBaseAddress + GPIO_OSPEEDR_OFFSET, speed, pinNumber);   
}

// Set mode for specified GPIO Pin
void GPIO_SetPortMode(volatile uint32_t *gpioBaseAddress, uint32_t mode, int pinNumber)
{
    if (pinNumber < 0 || pinNumber > 15)
        return;
    *gpioBaseAddress |= (mode << 2 * pinNumber);
}

// Set speed for specified GPIO Pin
void GPIO_SetPortSpeed(volatile uint32_t *gpioBaseAddress, uint32_t speed, int pinNumber)
{
    if (pinNumber < 0 || pinNumber > 15)
        return;
    *gpioBaseAddress |= (speed << 2 * pinNumber);
}

// Set speed for specified GPIO Pin
void GPIO_SetPortAF(volatile uint32_t *gpioBaseAddress, int altFunc,  int pinNumber)
{
    if (pinNumber < 0 || altFunc < 0 || pinNumber > 15 || altFunc > 15)
        return;
    *gpioBaseAddress |= (altFunc << 4 * pinNumber);
}

// Set bits for specified GPIO Pins
void GPIO_Set(volatile uint32_t * gpioAddress, uint32_t pins)
{
    if (pins <= ALL_PINS_OFF || pins > ALL_PINS_ON)
        return;
    *gpioAddress |= pins;
}

// Reset bits for specified GPIO Pins
void GPIO_Reset(volatile uint32_t * gpioAddress,uint32_t pins)
{
    if (pins <= ALL_PINS_OFF || pins > ALL_PINS_ON)
        return;
    *gpioAddress &= ~(pins);
}

// Set all bits for specified register
void GPIO_SetAll(volatile uint32_t * gpioAddress)
{
    *gpioAddress = ALL_PINS_ON;
}

// Set all bits for specified register
void GPIO_ResetAll(volatile uint32_t * gpioAddress)
{
    *gpioAddress = ALL_PINS_OFF;
}

// Toggle bits for specified GPIO pins
void GPIO_Toggle(volatile uint32_t * gpioAddress, uint32_t pins)
{
    if (pins <= ALL_PINS_OFF || pins > ALL_PINS_ON)
        return;
    *gpioAddress ^= pins;
}