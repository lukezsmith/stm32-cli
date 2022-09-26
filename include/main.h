int main(void);
static void prvSetupHardware(void);
static void prvInitGPIO(void);
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
void printAccelerometerData(void);
void ACCELERO_ReadAcc(void);

void Delay_ms(volatile int time_ms);


#define LED_ON_CMD                    1
#define LED_OFF_CMD                   2
#define LED_BLINK_ON_CMD              3
#define LED_BLINK_OFF_CMD             4
#define LED_LIGHT_SHOW_ON_CMD         5
#define LED_LIGHT_SHOW_OFF_CMD        6
#define PRINT_DATE_CMD                7

#define BTN_DEBOUNCE    10