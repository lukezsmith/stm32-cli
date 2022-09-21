#include <stdint.h>

// GPIO Pin constants
#define ALL_PINS_OFF    (uint32_t) 0
#define ALL_PINS_ON     (uint32_t) 0xffff

// GPIO Register offsets
#define GPIO_MODER_OFFSET   (uint32_t) 0x00
#define GPIO_OSPEEDR_OFFSET (uint32_t) 0x08

// GPIO Pin Modes
#define GPIO_MODE_INPUT     (uint32_t) 0
#define GPIO_MODE_OUTPUT    (uint32_t) 1
#define GPIO_MODE_AF        (uint32_t) 2
#define GPIO_MODE_ANALOG    (uint32_t) 3

// GPIO Pin Speeds
#define GPIO_SPEED_LOW          (uint32_t) 0x00
#define GPIO_SPEED_MEDIUM       (uint32_t) 0x01
#define GPIO_SPEED_HIGH         (uint32_t) 0x10
#define GPIO_SPEED_VERY_HIGH    (uint32_t) 0x11

void GPIO_Init(volatile uint32_t *gpioBaseAddress, int pinNumber, uint32_t mode, uint32_t speed );
void GPIO_SetPortMode(volatile uint32_t *gpioBaseAddress, uint32_t speed, int pinNumber);
void GPIO_SetPortSpeed(volatile uint32_t *gpioBaseAddress, uint32_t speed, int pinNumber);
void GPIO_SetPortAF(volatile uint32_t *gpioBaseAddress, int altFunc,  int pinNumber);
void GPIO_Set(volatile uint32_t * gpioAddress, uint32_t pins);
void GPIO_Reset(volatile uint32_t * gpioAddress,uint32_t pins);
void GPIO_SetAll(volatile uint32_t * gpioAddress);
void GPIO_ResetAll(volatile uint32_t * gpioAddress);
void GPIO_Toggle(volatile uint32_t * gpioAddress, uint32_t pins);
uint16_t convertGPIOPinNumberToBit(int pinNumber);