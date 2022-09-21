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
static void prvSetupHardware( void );
static void prvInitGPIO( void );
void ledBlinkTask ( void *p );
void UART2_SendChar (char c);
uint8_t UART2_GetChar (void);

TaskHandle_t ledBlinkTaskHandle = NULL;

UART_HandleTypeDef UART_Handler; /*Create UART_HandleTypeDef struct instance */
char Message[] = "Write anything on Serial Terminal\r\n"; /* Message to be transmitted through UART */

uint8_t buffer[30];
int indx = 0;
void UART2_SendChar (char c)
{
	/*********** STEPS FOLLOWED *************
	
	1. Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
		 for each data to be transmitted in case of single buffer.
	2. After writing the last data into the USART_DR register, wait until TC=1. This indicates
		 that the transmission of the last frame is complete. This is required for instance when
		 the USART is disabled or enters the Halt mode to avoid corrupting the last transmission.
	
	****************************************/

	USART2->DR = c;   // LOad the Data
	while (!(USART2->SR & (1<<6)));  // Wait for TC to SET.. This indicates that the data has been transmitted
}


uint8_t UART2_GetChar (void)
{
		/*********** STEPS FOLLOWED *************
	
	1. Wait for the RXNE bit to set. It indicates that the data has been received and can be read.
	2. Read the data from USART_DR  Register. This also clears the RXNE bit
	
	****************************************/
	uint8_t Temp;
	
	while (!(USART2->SR & (1<<5)));  // Wait for RXNE to SET.. This indicates that the data has been Received
	Temp = USART2->DR;  // Read the data. 
	return Temp;
}

int main(void)
{
	// HAL_Init(); /* HAL library initialization */
	// UART2_Configuration(); /* Call UART2 initialization define below */
	// HAL_UART_Transmit(&UART_Handler, (uint8_t *)Message, strlen(Message), 10);
	// while(1)
	// {
	// 	 uint8_t buffer[4];
    //  HAL_UART_Receive(&UART_Handler, buffer, sizeof(buffer), HAL_MAX_DELAY);
    //  HAL_UART_Transmit(&UART_Handler, buffer, sizeof(buffer), HAL_MAX_DELAY);
	// }

	// Setup hardware 
    prvSetupHardware();

	UART_Transmit(&USART2->DR, 'H');
	// UART_Transmit(&USART2->DR, 'e');
	// UART_Transmit(&USART2->DR, 'l');
	// UART_Transmit(&USART2->DR, 'l');
	// UART_Transmit(&USART2->DR, 'o');
	// UART_Transmit(&USART2->DR, '!');
	while(1)
	{
		int c = UART2_GetChar();
		UART2_SendChar (c);
		// UART_Transmit(&USART2->DR, 'H');
		// USART2->DR = 'H';
		// delay (100000000);
		// Delay_ms(100);


	// 	 uint8_t buffer[4];
    //  HAL_UART_Receive(&UART_Handler, buffer, sizeof(buffer), HAL_MAX_DELAY);
    //  HAL_UART_Transmit(&UART_Handler, buffer, sizeof(buffer), HAL_MAX_DELAY);
	}

    // create tasks
    // xTaskCreate(ledBlinkTask, "LED", configMINIMAL_STACK_SIZE, (void *) NULL, tskIDLE_PRIORITY, &ledBlinkTaskHandle);

    // // start scheduler
    // vTaskStartScheduler();

    // for ( ;; );
}

void ledBlinkTask ( void *p )
{
    TickType_t xLastWakeTime;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
		GPIO_Toggle(&GPIOD->ODR, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);// Toggle LEDs
    }
}

void prvInitClocks( void )
{
	// Enable Clock for GPIOD (bit 3)
	RCC->AHB1ENR |= (1 << 3);

	// Enable Clock for GPIOA (bit 3)
	RCC->AHB1ENR |= (1 << 0);

	// Enable clock for USART2
	// RCC->APB1ENR |= RCC_APB1ENR_USART2EN
	RCC->APB1ENR |= (1 << 17);

}

void prvInitGPIO( void )
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

void prvSetupHardware( void )
{	
	// Setup clocks
	prvInitClocks();

    // setup GPIO outputs
    prvInitGPIO();

	// setup UART
	prvInitUART();
}


void vApplicationMallocFailedHook( void )
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
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
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

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

void prvInitUART(void)
{
	// USART2->CR1 = 0x00;   // Clear ALL
	// Init UART2 with 8 length word, 0 stop bits and 11520 Baud Rate
	UART_Init(USART2, 8, 0, 11520);
	
	// Enable RX, TX
	UART_EnableRX(&USART2->CR1);
	UART_EnableTX(&USART2->CR1);
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
        for(j = 0; j < time_ms*4000; j++)
            {}  /* excute NOP for 1ms */
}
