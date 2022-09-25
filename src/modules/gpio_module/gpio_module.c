#include "gpio_module.h"
// #include "stm32f4xx.h"

// convert specified pin to bits
uint16_t convertGPIOPinNumberToBit(int pinNumber)
{
    if (pinNumber <= 0 || pinNumber > 16)
        return 0;
    return 1 << (pinNumber - 1);
}

// function to initialise GPIO pin(s)
void GPIO_Init(volatile uint32_t *gpioBaseAddress, int pinNumber, uint32_t mode, uint32_t speed)
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
void GPIO_SetPortAF(volatile uint32_t *gpioBaseAddress, int altFunc, int pinNumber)
{
    if (pinNumber < 0 || altFunc < 0 || pinNumber > 15 || altFunc > 15)
        return;
    *gpioBaseAddress |= (altFunc << 4 * pinNumber);
}

// Set bits for specified GPIO Pins
void GPIO_Set(volatile uint32_t *gpioAddress, uint32_t pins)
{
    if (pins <= ALL_PINS_OFF || pins > ALL_PINS_ON)
        return;
    *gpioAddress |= pins;
}

// Reset bits for specified GPIO Pins
void GPIO_Reset(volatile uint32_t *gpioAddress, uint32_t pins)
{
    if (pins <= ALL_PINS_OFF || pins > ALL_PINS_ON)
        return;
    *gpioAddress &= ~(pins);
}

// Set all bits for specified register
void GPIO_SetAll(volatile uint32_t *gpioAddress)
{
    *gpioAddress = ALL_PINS_ON;
}

// Set all bits for specified register
void GPIO_ResetAll(volatile uint32_t *gpioAddress)
{
    *gpioAddress = ALL_PINS_OFF;
}

// Toggle bits for specified GPIO pins
void GPIO_Toggle(volatile uint32_t *gpioAddress, uint32_t pins)
{
    if (pins <= ALL_PINS_OFF || pins > ALL_PINS_ON)
        return;
    *gpioAddress ^= pins;
}

// Toggle bits for specified GPIO pins
int GPIO_Read(volatile uint32_t *gpioAddress, int pinNumber)
{
    if (pinNumber < 0 || pinNumber > 15)
        return -1;
    return *gpioAddress & (1 << pinNumber);
}

// void GPIO_ConfigureInterrupt(volatile uint32_t *portAddress, int pinNumber, edge_select edge)
// {
//     if (portAddress == GPIOA)
//     {
//         switch(pinNumber)
//         {
//             case 0: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA;
//                 break;
//             case 1: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA;
//                 break;
//             case 2: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI2_PA;
//                 break;
//             case 3: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI3_PA;
//                 break;
//             case 4: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI4_PA;
//                 break;
//             case 5: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI5_PA;
//                 break;
//             case 6: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI6_PA;
//                 break;
//             case 7: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI7_PA;
//                 break;
//             case 8: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI8_PA;                                                                                                                
//                 break;
//             case 9: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI9_PA;
//                 break;
//             case 10: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI10_PA;
//                 break;
//             case 11: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI11_PA;
//                 break;
//             case 12: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI12_PA;
//                 break;
//             case 13: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI13_PA;
//                 break;
//             case 14: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI14_PA;
//                 break;
//             case 15: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI15_PA;                
//                 break;
//         }
//     }

//     if (portAddress == GPIOB)
//     {
//                 switch(pinNumber)
//         {
//             case 0: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PB;
//                 break;
//             case 1: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PB;
//                 break;
//             case 2: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI2_PB;
//                 break;
//             case 3: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI3_PB;
//                 break;
//             case 4: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI4_PB;
//                 break;
//             case 5: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI5_PB;
//                 break;
//             case 6: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI6_PB;
//                 break;
//             case 7: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI7_PB;
//                 break;
//             case 8: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI8_PB;                                                                                                                
//                 break;
//             case 9: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI9_PB;
//                 break;
//             case 10: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI10_PB;
//                 break;
//             case 11: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI11_PB;
//                 break;
//             case 12: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI12_PB;
//                 break;
//             case 13: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI13_PB;
//                 break;
//             case 14: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI14_PB;
//                 break;
//             case 15: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI15_PB;                
//                 break;
//         }
//     }

//     if (portAddress == GPIOC)
//     {
//                 switch(pinNumber)
//         {
//             case 0: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PC;
//                 break;
//             case 1: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PC;
//                 break;
//             case 2: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI2_PC;
//                 break;
//             case 3: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI3_PC;
//                 break;
//             case 4: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI4_PC;
//                 break;
//             case 5: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI5_PC;
//                 break;
//             case 6: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI6_PC;
//                 break;
//             case 7: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI7_PC;
//                 break;
//             case 8: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI8_PC;                                                                                                                
//                 break;
//             case 9: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI9_PC;
//                 break;
//             case 10: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI10_PC;
//                 break;
//             case 11: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI11_PC;
//                 break;
//             case 12: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI12_PC;
//                 break;
//             case 13: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI13_PC;
//                 break;
//             case 14: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI14_PC;
//                 break;
//             case 15: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI15_PC;                
//                 break;
//         }
//     }

//     if (portAddress == GPIOD)
//     {
//                 switch(pinNumber)
//         {
//             case 0: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PD;
//                 break;
//             case 1: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PD;
//                 break;
//             case 2: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI2_PD;
//                 break;
//             case 3: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI3_PD;
//                 break;
//             case 4: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI4_PD;
//                 break;
//             case 5: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI5_PD;
//                 break;
//             case 6: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI6_PD;
//                 break;
//             case 7: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI7_PD;
//                 break;
//             case 8: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI8_PD;                                                                                                                
//                 break;
//             case 9: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI9_PD;
//                 break;
//             case 10: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI10_PD;
//                 break;
//             case 11: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI11_PD;
//                 break;
//             case 12: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI12_PD;
//                 break;
//             case 13: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI13_PD;
//                 break;
//             case 14: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI14_PD;
//                 break;
//             case 15: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI15_PD;                
//                 break;
//         }
//     }

//     if (portAddress == GPIOE)
//     {
//                 switch(pinNumber)
//         {
//             case 0: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PE;
//                 break;
//             case 1: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PE;
//                 break;
//             case 2: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI2_PE;
//                 break;
//             case 3: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI3_PE;
//                 break;
//             case 4: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI4_PE;
//                 break;
//             case 5: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI5_PE;
//                 break;
//             case 6: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI6_PE;
//                 break;
//             case 7: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI7_PE;
//                 break;
//             case 8: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI8_PE;                                                                                                                
//                 break;
//             case 9: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI9_PE;
//                 break;
//             case 10: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI10_PE;
//                 break;
//             case 11: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI11_PE;
//                 break;
//             case 12: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI12_PE;
//                 break;
//             case 13: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI13_PE;
//                 break;
//             case 14: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI14_PE;
//                 break;
//             case 15: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI15_PE;                
//                 break;
//         }
//     }

//     if (portAddress == GPIOF)
//     {
//                 switch(pinNumber)
//         {
//             case 0: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PF;
//                 break;
//             case 1: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PF;
//                 break;
//             case 2: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI2_PF;
//                 break;
//             case 3: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI3_PF;
//                 break;
//             case 4: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI4_PF;
//                 break;
//             case 5: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI5_PF;
//                 break;
//             case 6: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI6_PF;
//                 break;
//             case 7: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI7_PF;
//                 break;
//             case 8: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI8_PF;                                                                                                                
//                 break;
//             case 9: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI9_PF;
//                 break;
//             case 10: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI10_PF;
//                 break;
//             case 11: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI11_PF;
//                 break;
//             case 12: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI12_PF;
//                 break;
//             case 13: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI13_PF;
//                 break;
//             case 14: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI14_PF;
//                 break;
//             case 15: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI15_PF;                
//                 break;
//         }
//     }
//     if (portAddress == GPIOG)
//     {
//                 switch(pinNumber)
//         {
//             case 0: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PG;
//                 break;
//             case 1: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PG;
//                 break;
//             case 2: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI2_PG;
//                 break;
//             case 3: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI3_PG;
//                 break;
//             case 4: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI4_PG;
//                 break;
//             case 5: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI5_PG;
//                 break;
//             case 6: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI6_PG;
//                 break;
//             case 7: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI7_PG;
//                 break;
//             case 8: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI8_PG;                                                                                                                
//                 break;
//             case 9: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI9_PG;
//                 break;
//             case 10: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI10_PG;
//                 break;
//             case 11: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI11_PG;
//                 break;
//             case 12: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI12_PG;
//                 break;
//             case 13: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI13_PG;
//                 break;
//             case 14: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI14_PG;
//                 break;
//             case 15: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI15_PG;                
//                 break;
//         }
//     }

//     if (portAddress == GPIOH)
//     {
//                 switch(pinNumber)
//         {
//             case 0: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PH;
//                 break;
//             case 1: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PH;
//                 break;
//             case 2: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI2_PH;
//                 break;
//             case 3: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI3_PH;
//                 break;
//             case 4: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI4_PH;
//                 break;
//             case 5: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI5_PH;
//                 break;
//             case 6: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI6_PH;
//                 break;
//             case 7: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR2_EXTI7_PH;
//                 break;
//             case 8: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI8_PH;                                                                                                                
//                 break;
//             case 9: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI9_PH;
//                 break;
//             case 10: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI10_PH;
//                 break;
//             case 11: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR3_EXTI11_PH;
//                 break;
//             case 12: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI12_PH;
//                 break;
//             case 13: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI13_PH;
//                 break;
//             case 14: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI14_PH;
//                 break;
//             case 15: 
//                 SYSCFG->EXTICR[0] = SYSCFG_EXTICR4_EXTI15_PH;
//                 break;
//         }
//     }

//     if(edge == RISING_EDGE)
//     {
//         EXTI->RTSR |= (1 << pinNumber);
//     }

//     if(edge == FALLING_EDGE)
//     {
//         EXTI->FTSR |= (1 << pinNumber);
//     }
//     if(edge == RISING_FALLING_EDGE)
//     {
//         EXTI->FTSR |= (1 << pinNumber);
//         EXTI->RTSR |= (1 << pinNumber);
//     }
       
// }

void GPIO_EnableInterrupt(volatile uint32_t *portAddress, int irqNumber)
{

}