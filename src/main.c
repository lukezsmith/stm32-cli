#include <string.h>
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "task.h"
#include "gpio_module.h"


/*Function prototype for delay and UART2 configuration functions */
void UART2_Configuration(void);
void Delay_ms(volatile int time_ms);

int main(void);
static void prvSetupHardware( void );
static void prvInitGPIO( void );
void ledBlinkTask ( void *p );

TaskHandle_t ledBlinkTaskHandle = NULL;

UART_HandleTypeDef UART_Handler; /*Create UART_HandleTypeDef struct instance */
char Message[] = "Write anything on Serial Terminal\r\n"; /* Message to be transmitted through UART */

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

    // create tasks
    xTaskCreate(ledBlinkTask, "LED", configMINIMAL_STACK_SIZE, (void *) NULL, tskIDLE_PRIORITY, &ledBlinkTaskHandle);

    // start scheduler
    vTaskStartScheduler();

    for ( ;; );
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

void prvInitGPIO( void )
{
	GPIO_InitTypeDef GPIOD_Params; // Initilisation structure for GPIOD Settings
	
	__HAL_RCC_GPIOD_CLK_ENABLE(); // Turn on Clock of GPIOD
	
	// Configure the GPIO Pins 12, 13, 14 and 15 used for LEDs
	GPIOD_Params.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; // Select pins 12 to 15
	GPIOD_Params.Mode = GPIO_MODE_OUTPUT_PP; // Set pins to push pull output mode
	GPIOD_Params.Speed = GPIO_SPEED_LOW; // Set low output speed
	HAL_GPIO_Init(GPIOD, &GPIOD_Params); // Initialise GPIOD according to parameters on GPIOD_Params
}

void prvSetupHardware( void )
{
    // setup STM32 clock, PLL and Flash
    SystemInit();

    // setup GPIO outputs
    prvInitGPIO();
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


void UART2_Configuration(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE(); /* Enable clock to PORTA - UART2 pins PA2 and PA3 */
	__HAL_RCC_USART2_CLK_ENABLE(); /* Enable clock to UART2 module */
	
	GPIO_InitTypeDef UART2_GPIO_Handler; /*Create GPIO_InitTypeDef struct instance */
	UART2_GPIO_Handler.Pin = GPIO_PIN_2 | GPIO_PIN_3; 
	UART2_GPIO_Handler.Mode = GPIO_MODE_AF_PP;
	UART2_GPIO_Handler.Pull = GPIO_PULLUP;
	UART2_GPIO_Handler.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	UART2_GPIO_Handler.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &UART2_GPIO_Handler);
	//UART Configuration
	UART_Handler.Instance = USART2;
	UART_Handler.Init.BaudRate = 115200;
	UART_Handler.Init.Mode = UART_MODE_TX_RX;
	UART_Handler.Init.WordLength = UART_WORDLENGTH_8B;
	UART_Handler.Init.StopBits = UART_STOPBITS_1;
	UART_Handler.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&UART_Handler);	
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