#include <string.h>
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "task.h"
#include "gpio_module.h"
#include "uart_module.h"

/*Function prototype for delay and UART2 configuration functions */
void UART2_Configuration(void);
void Delay_ms(volatile int time_ms);

int main(void);
static void prvSetupHardware(void);
static void prvInitGPIO(void);
void ledBlinkTask(void *p);
void uartTask(void *p);
void uartTaskPolling(void *p);
void UART2_SendChar(char c);
uint8_t UART2_GetChar(void);

TaskHandle_t ledBlinkTaskHandle = NULL;
TaskHandle_t uartTaskHandle = NULL;

UART_HandleTypeDef UART_Handler;						  /*Create UART_HandleTypeDef struct instance */
char Message[] = "Write anything on Serial Terminal\r\n"; /* Message to be transmitted through UART */

int main(void)
{
	// Setup hardware
	prvSetupHardware();

	// Greeting message
	// UART_TransmitString(USART2, "Hello!\r\n");

	// UART Polling
	// while(1)
	// {
	// 	// char c = UART_Receive(USART2);
	// 	// UART_Transmit(USART2, c);

	// }

	// create tasks
	xTaskCreate(ledBlinkTask, "LED", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, &ledBlinkTaskHandle);
	xTaskCreate(uartTask, "UART", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, &uartTaskHandle);
	// xTaskCreate(uartTaskPolling, "UART", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, &uartTaskHandle);

	// start scheduler
	vTaskStartScheduler();

	// for (;;)
	// 	;
	while(1);
	return 0;
}

// Tasks

void ledBlinkTask(void *p)
{
	TickType_t xLastWakeTime;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
		// GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15); // Toggle LEDs
		GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15); // Toggle LEDs
	}
}

void uartTaskPolling(void *p)
{
	UART_TransmitString(USART2, "Hello!\r\n");
	while(1)
	{
		char c = UART_Receive(USART2);
		UART_Transmit(USART2, c);

	}

}

void uartTask(void *p)
{
	UART_TransmitString(USART2, "uartTask running!");
	const TickType_t xBlockTime = pdMS_TO_TICKS(100);
	uint32_t ulNotifiedValue;
	// for (;;)
	while(1)
	{
		// vTaskSuspend(NULL);
		// GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15); // Toggle LEDs
		// UART_TransmitString(USART2, "uartTask running!");
		ulNotifiedValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (ulNotifiedValue)
		{
			GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_12);
			// UART_Transmit(USART2, "!");
			// Handle UART RX
			if (USART2->SR & USART_SR_RXNE)
			{
				// char c = UART_Receive(USART2);
				// UART_Transmit(USART2, c);
			}
			// // Handle UART TX
			// if (USART2->SR & USART_SR_TXE)
			// {
			// }
		}
		else
		{
		}
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
	GPIO_Init(GPIOD, 12, GPIO_MODE_OUTPUT, GPIO_SPEED_LOW);
	GPIO_Init(GPIOD, 13, GPIO_MODE_OUTPUT, GPIO_SPEED_LOW);
	GPIO_Init(GPIOD, 14, GPIO_MODE_OUTPUT, GPIO_SPEED_LOW);
	GPIO_Init(GPIOD, 15, GPIO_MODE_OUTPUT, GPIO_SPEED_LOW);

	// init UART Pins
	GPIO_Init(GPIOA, 2, GPIO_MODE_AF, GPIO_SPEED_LOW);
	GPIO_Init(GPIOA, 3, GPIO_MODE_AF, GPIO_SPEED_LOW);

	// Set Alternate function low register for pins 2,3
	GPIO_SetPortAF(&GPIOA->AFR[0], 7, 2);
	GPIO_SetPortAF(&GPIOA->AFR[0], 7, 3);
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
	UART_Init(USART2, 8, 0, 11520);

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
	// BaseType_t xHigherPriorityTaskWoken;

	// UART_TransmitString(USART2, "\r\nIRQHandler!\r\n");

	// // Clear interrupt
	// // prvClearInterruptSource();
	// // taskDISABLE_INTERRUPTS();
	// // NVIC_ClearPendingIRQ(USART2_IRQn);
	// // NVIC_DisableIRQ(USART2_IRQn);


	// xHigherPriorityTaskWoken = pdFALSE;
	// // xHigherPriorityTaskWoken = pdTRUE;

	// // Unblock the task
	// vTaskNotifyGiveFromISR(uartTask, &xHigherPriorityTaskWoken);

	// // xHigherPriorityTaskWoken = xTaskResumeFromISR(uartTaskHandle);

	// // Force context switch
	// portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	BaseType_t rxReceived;
	BaseType_t txReceived;
	// Handle UART RX
		if (USART2->SR & USART_SR_RXNE)
		{
			char c = UART_Receive(USART2);
			UART_Transmit(USART2, c);
			rxReceived = pdTRUE;
			GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_12);

		}
		// Handle UART TX
		if (USART2->SR & USART_SR_TXE)
		{
			txReceived = pdTRUE;

		}
		portYIELD_FROM_ISR(rxReceived || txReceived);
}

// void SysTick_Handler(void)
// {
//   HAL_IncTick();
//   HAL_SYSTICK_IRQHandler();
// }
/*Generate ms */
void Delay_ms(volatile int time_ms)
{
	int j;
	for (j = 0; j < time_ms * 4000; j++)
	{
	} /* excute NOP for 1ms */
}
