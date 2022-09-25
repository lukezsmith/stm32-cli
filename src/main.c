#include <string.h>
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "utils.h"
#include "gpio_module.h"
#include "uart_module.h"

TaskHandle_t ledBlinkOnTaskHandle = NULL;
TaskHandle_t ledLightShowTaskHandle = NULL;
TaskHandle_t uartTaskHandle = NULL;
TaskHandle_t menuDisplayTaskHandle = NULL;
TaskHandle_t uartWriteTaskHandle = NULL;
TaskHandle_t rxCmdTaskHandle = NULL;
TaskHandle_t processCmdTaskHandle = NULL;

QueueHandle_t uartQueue;
QueueHandle_t cmdQueue;

uint8_t cmdBuffer[20];
uint8_t cmdLen = 0;
uint8_t byte = 0;

int uartQueueItemSize = 300;
int cmdQueueItemSize = 10;

uint16_t buttonPressed = 3;
uint16_t buttonPressed_0 = 0;
uint16_t buttonPressed_1 = 0;
uint16_t statoButtonPressed = 3;

// Menu string to display menu to user
char menu[] = {"\
\r\n============ stm32-cli ============\
\r\nLED ON		             ----> 1\
\r\nLED OFF		             ----> 2\
\r\nLED BLINK ON		     ----> 3\
\r\nLED BLINK OFF		     ----> 4\
\r\nLED LIGHT SHOW ON	     ----> 5\
\r\nLED LIGHT SHOW OFF	     ----> 6\
\r\nOr, press button for IMU data.\
\r\n\nType your option here: "};

int main(void)
{
	// Setup hardware
	prvSetupHardware();

	// create queues
	uartQueue = xQueueCreate(10, uartQueueItemSize);
	cmdQueue = xQueueCreate(10, sizeof(char));

	// create tasks
	// Priority 0 to enqueue menu first
	xTaskCreate(menuDisplayTask, "MENU", configMINIMAL_STACK_SIZE, (void *)NULL, 1, &menuDisplayTaskHandle);
	xTaskCreate(rxCmdTask, "RXCMD", configMINIMAL_STACK_SIZE, (void *)NULL, 2, &rxCmdTaskHandle);
	xTaskCreate(processCmdTask, "PROCESSCMD", configMINIMAL_STACK_SIZE, (void *)NULL, 2, &processCmdTaskHandle);
	xTaskCreate(uartWriteTask, "UART", configMINIMAL_STACK_SIZE, (void *)NULL, 2, &uartWriteTaskHandle);

	// start scheduler
	vTaskStartScheduler();

	for (;;);
}

// Tasks

// Enqueues menu string to UART transmit buffer
void menuDisplayTask(void *p)
{
	for (;;)
	{
		// enqueue menu without blocking
		xQueueSend(uartQueue, &menu, (TickType_t)0);

		// block until notified to enqueue menu again
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
	}
}

// Transmits enqueued data to UART
void uartWriteTask(void *p)
{
	char rxBuff[uartQueueItemSize];

	for (;;)
	{
		// check queue is not empty
		if (uartQueue != 0)
		{
			if (xQueueReceive(uartQueue, (void *)rxBuff, (TickType_t)5))
			{
				UART_TransmitString((volatile uint32_t *)USART2, rxBuff);
				vTaskDelay(1000);
			}
		}
	}
}
// Receives user command
void rxCmdTask(void *p)
{
	uint8_t newCmd;

	for (;;)
	{
		// Wait for notification
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

		// critical section, disable interrupts
		taskENTER_CRITICAL();

		// parse buffer for number
		uint8_t pos = 0;
		while (cmdBuffer[pos] == '\r' || cmdBuffer[pos] == '\n')
		{
			pos++;
		}

		// get command code
		newCmd = getCommandCode(&cmdBuffer[pos]);

		// re-enable interrupts
		taskEXIT_CRITICAL();

		// add command to queue
		xQueueSend(cmdQueue, &newCmd, (TickType_t)0);

		// force context switch
		portYIELD();
	}
}

// Receives user command
void processCmdTask(void *p)
{
	int newCmd;
	char rxBuff[1];

	for (;;)
	{
		// check queue is not empty
		if (cmdQueue != 0)
		{
			// if there's a command queued
			if (xQueueReceive(cmdQueue, (void *)rxBuff, portMAX_DELAY))
			{
				newCmd = rxBuff[0];
				switch (newCmd)
				{
				case LED_ON_CMD:
					ledOnCmd();
					break;
				case LED_OFF_CMD:
					ledOffCmd();
					break;
				case LED_BLINK_ON_CMD:
					xTaskCreate(ledBlinkOnTask, "LED", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, &ledBlinkOnTaskHandle);
					char blinkMsg[] = "\nLED blink started.\r\n";
					xQueueSend(uartQueue, &blinkMsg, (TickType_t)0);
					break;
				case LED_BLINK_OFF_CMD:
					ledBlinkTaskSuspend();
					break;
				case LED_LIGHT_SHOW_ON_CMD:
					xTaskCreate(ledLightShowTask, "LED", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, &ledLightShowTaskHandle);
					char showMsg[] = "\r\nLED light show started.\r\n";
					xQueueSend(uartQueue, &showMsg, (TickType_t)0);
					break;
				case LED_LIGHT_SHOW_OFF_CMD:
					ledLightShowTaskSuspend();
					break;
				default:
					printErrorMessage();
					break;
				}
			}
		}
	}
}

// Task that blinks LED
void ledBlinkOnTask(void *p)
{
	// Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
		// GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15); // Toggle LEDs
		GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_12); // Toggle LEDs
	}
}

// Task that blinks LED
void ledLightShowTask(void *p)
{
	// Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_12); // Toggle LEDs
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
		GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_12); // Toggle LEDs
		GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_13); // Toggle LEDs
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
		GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_13); // Toggle LEDs
		GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_14); // Toggle LEDs
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
		GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_14); // Toggle LEDs
		GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_15); // Toggle LEDs
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
		GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_15); // Toggle LEDs
	}
}

void ledOnCmd(void)
{
	GPIO_Set(&GPIOD->ODR, GPIO_PIN_12);
	char msg[] = "\nLED turned on.\r\n";
	xQueueSend(uartQueue, &msg, (TickType_t)0);
}

void ledOffCmd(void)
{
	GPIO_Reset(&GPIOD->ODR, GPIO_PIN_12);
	char msg[] = "\nLED turned off.\r\n";
	xQueueSend(uartQueue, &msg, (TickType_t)0);
}

void ledBlinkTaskSuspend(void)
{
	// suspend task (if running)
	if (ledBlinkOnTaskHandle != NULL)
	{
		vTaskSuspend(ledBlinkOnTaskHandle);

		// turn leds off
		GPIO_Reset(&GPIOD->ODR, GPIO_PIN_12);

		char msg[] = "\nLED blink stopped.\r\n";
		xQueueSend(uartQueue, &msg, (TickType_t)0);
	}
}

void ledLightShowTaskSuspend(void)
{
	// suspend task (if running)
	if (ledLightShowTaskHandle != NULL)
	{
		vTaskSuspend(ledLightShowTaskHandle);

		// turn leds off
		GPIO_Reset(&GPIOD->ODR, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

		char msg[] = "\nLED light show stopped.\r\n";
		xQueueSend(uartQueue, &msg, (TickType_t)0);
	}
}

void prvInitClocks(void)
{
	// Enable Clock for GPIOD (bit 3)
	RCC->AHB1ENR |= (1 << 3);

	// Enable Clock for GPIOA (bit 3)
	RCC->AHB1ENR |= (1 << 0);

	// Enable clock for USART2
	// RCC->APB1ENR |= RCC_APB1ENR_USART2EN
	RCC->APB1ENR |= (1 << 17);
}

void prvInitGPIO(void)
{
	// init LED GPIOD Pins 12-15
	GPIO_Init((volatile uint32_t *)GPIOD, 12, GPIO_PORT_MODE_OUTPUT, GPIO_PORT_SPEED_LOW);
	GPIO_Init((volatile uint32_t *)GPIOD, 13, GPIO_PORT_MODE_OUTPUT, GPIO_PORT_SPEED_LOW);
	GPIO_Init((volatile uint32_t *)GPIOD, 14, GPIO_PORT_MODE_OUTPUT, GPIO_PORT_SPEED_LOW);
	GPIO_Init((volatile uint32_t *)GPIOD, 15, GPIO_PORT_MODE_OUTPUT, GPIO_PORT_SPEED_LOW);

	// Init User button GPIOA0
	GPIO_Init((volatile uint32_t *)GPIOA, 0, GPIO_PORT_MODE_INPUT, GPIO_PORT_SPEED_LOW);

	// init UART Pins
	GPIO_Init((volatile uint32_t *)GPIOA, 2, GPIO_PORT_MODE_AF, GPIO_PORT_SPEED_LOW);
	GPIO_Init((volatile uint32_t *)GPIOA, 3, GPIO_PORT_MODE_AF, GPIO_PORT_SPEED_LOW);

	// Set Alternate function low register for pins 2,3
	GPIO_SetPortAF(&GPIOA->AFR[0], 7, 2);
	GPIO_SetPortAF(&GPIOA->AFR[0], 7, 3);

	// Enable interrupts on PA0
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	EXTI->FTSR |= (1 << 0);
	EXTI->IMR |= (1 << 0);
	NVIC_EnableIRQ(EXTI0_IRQn);
}

void prvSetupHardware(void)
{
	// Setup clocks
	prvInitClocks();

	// setup GPIO outputs
	prvInitGPIO();

	// setup UART
	prvInitUART();
}

void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}

void prvInitUART(void)
{
	// USART2->CR1 = 0x00;   // Clear ALL
	// Init UART2 with 8 length word, 0 stop bits and 11520 Baud Rate
	UART_Init((volatile uint32_t *)USART2, 8, 0, 11520);

	// Enable RX, TX
	UART_EnableRX(&USART2->CR1);
	UART_EnableTX(&USART2->CR1);

	// Enable RXNE and TXE interrupts
	UART_EnableRXNEIE(&USART2->CR1);
	// UART_EnableTXEIE(&USART2->CR1);

	// Enable interrupts for USART2
	NVIC_SetPriority(USART2_IRQn, 5);
	NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Receive byte
	byte = UART_Receive((volatile uint32_t *)USART2);

	cmdBuffer[cmdLen++] = byte & 0xFF;

	// check if received byte is user pressing enter
	if (byte == '\r')
	{
		// display response
		UART_TransmitString((volatile uint32_t *)USART2, "\rYour command is: \"");
		UART_Transmit((volatile uint32_t *)USART2, cmdBuffer[0]);
		UART_TransmitString((volatile uint32_t *)USART2, "\".\r\n");

		// reset cmdLen
		cmdLen = 0;

		// notify cmd handling task
		xTaskNotifyFromISR(menuDisplayTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
		xTaskNotifyFromISR(rxCmdTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
	}

	if (xHigherPriorityTaskWoken)
	{
		taskYIELD();
	}
}


void EXTI0_IRQHandler (void)
{
	// clear interrupt
	EXTI->PR |= (1<<0);

	buttonPressed = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	if(buttonPressed == 0)
	{
		buttonPressed_0++;
		buttonPressed_1 = 0;
		if(buttonPressed_0 >= BTN_DEBOUNCE)
		{
			buttonPressed_0 = BTN_DEBOUNCE+1;
			statoButtonPressed = 0;
		}
	}
	else{
		buttonPressed_0 = 0;
		buttonPressed_1++;
		if(buttonPressed_1 >= BTN_DEBOUNCE){
			buttonPressed_1 = BTN_DEBOUNCE+1;
			statoButtonPressed = 1;
		}
	}
	if(statoButtonPressed)
	{

	UART_Transmit((volatile uint32_t *)USART2, '1');
	}else{
		UART_Transmit((volatile uint32_t *)USART2, '0');
	}
	// 	UART_TransmitString(USART2, "YES!!!!");
	// 	buttonPressed = 0;
	// 	// if(GPIO_Read(GPIOA, 0))
	// 	UART_TransmitString(USART2, GPIO_Read(GPIOA, 0));
	// }
}