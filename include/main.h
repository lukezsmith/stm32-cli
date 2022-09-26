#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "task.h"
#include "queue.h"
#include "utils.h"
#include "gpio_module.h"
#include "uart_module.h"
#include "semphr.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_accelerometer.h"

int main(void);
void prvSetupHardware(void);
void prvInitGPIO(void);
void prvInitUART(void);
void ledBlinkOnTask(void *p);
void ledLightShowTask(void *p);
void uartTask(void *p);
void uartWriteTask(void *p);
void menuDisplayTask(void *p);
void rxCmdTask(void *p);
void processCmdTask(void *p);
void vButtonTask(void *p);
void ledOnCmd(void);
void ledOffCmd(void);
void ledBlinkOnCmd(void);
void ledBlinkTaskSuspend(void);
void ledLightShowTaskSuspend(void);
void ACCEL_Read(void);


#define LED_ON_CMD                    1
#define LED_OFF_CMD                   2
#define LED_BLINK_ON_CMD              3
#define LED_BLINK_OFF_CMD             4
#define LED_LIGHT_SHOW_ON_CMD         5
#define LED_LIGHT_SHOW_OFF_CMD        6
#define PRINT_DATE_CMD                7

#define butDEBOUNCE_DELAY (200 / portTICK_RATE_MS)

#define UART_TRANSMIT_DELAY 750