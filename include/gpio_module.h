#include <stdint.h>

extern const uint16_t ALL_PINS_OFF;
extern const uint16_t ALL_PINS_ON;

void GPIO_Set(uint32_t * gpioAddress, uint16_t pins);

void GPIO_Reset(uint32_t * gpioAddress,uint16_t pins);

void GPIO_SetAll(uint32_t * gpioAddress);

void GPIO_ResetAll(uint32_t * gpioAddress);

void GPIO_Toggle(volatile uint32_t * gpioAddress, uint16_t pins);

uint16_t convertGPIOPinNumberToBit(int pinNumber);