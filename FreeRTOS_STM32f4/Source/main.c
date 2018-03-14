//******************************************************************************
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"
//******************************************************************************

//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "timers.h"
//******************************************************************************

void vBlueTimerCallback(TimerHandle_t xTimer);
void vRedTimerCallback(TimerHandle_t xTimer);
void vGreenTimerCallback(TimerHandle_t xTimer);
void vOrangeTimerCallback(TimerHandle_t xTimer);

void vLedBlinkBlue(void *pvParameters);
void vLedBlinkRed(void *pvParameters);
void vLedBlinkGreen(void *pvParameters);
void vLedBlinkOrange(void *pvParameters);
void vLongPressEvent(void *pvParameters);

void detectButtonPress(void *pvParameters);

void shortPressEvent();
void longPressEvent();

void initButton();
//void Delay(uint32_t val);

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

#define BLINK_TIME	1000

#define BLUE_MAX_COUNT	4
#define RED_MAX_COUNT	6
#define ORANGE_MAX_COUNT 8
#define GREEN_MAX_COUNT 10

#define BLUE_TIMER	0
#define RED_TIMER	1
#define ORANGE_TIMER	2
#define	GREEN_TIMER	3

#define NUM_TIMERS	5
#define NUM_TASKS 5
TimerHandle_t xTimers[NUM_TIMERS];
TaskHandle_t xTasks[NUM_TASKS];

#define BOUNCE_THRESHOLD 30
#define LONG_PRESS_THRESHOLD 300

//******************************************************************************
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32f4xx.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32f4xx.c file
	 */
	
	/*!< Most systems default to the wanted configuration, with the noticeable 
		exception of the STM32 driver library. If you are using an STM32 with 
		the STM32 driver library then ensure all the priority bits are assigned 
		to be preempt priority bits by calling 
		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); before the RTOS is started.
	*/
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	
	xTimers[0] = xTimerCreate((const char*)"Blue timer", pdMS_TO_TICKS(1000), pdTRUE, ( void * ) 0, vBlueTimerCallback);
	xTimers[4] = xTimerCreate((const char*)"Long Press timer", pdMS_TO_TICKS(LONG_PRESS_THRESHOLD), pdFALSE, ( void * ) 0, vLongPressEvent);
	
	xTaskCreate( vLedBlinkBlue, (const char*)"Led Blink Task Blue", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, xTasks[0] );
	xTaskCreate( vLedBlinkRed, (const char*)"Led Blink Task Red", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, xTasks[1] );
	xTaskCreate( vLedBlinkGreen, (const char*)"Led Blink Task Green", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, xTasks[2] );
	xTaskCreate( vLedBlinkOrange, (const char*)"Led Blink Task Orange", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, xTasks[3] );
	xTaskCreate ( detectButtonPress, (const char*)"Detect Button Press",
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, xTasks[4] );
	
	vTaskStartScheduler();
}

void vBlueTimerCallback( TimerHandle_t xTimer) {
	uint32_t ulCount;
	
	ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
	ulCount++;
	
	if (ulCount >= BLUE_MAX_COUNT) {
		xTimerStop(xTimer, 0 );
		// Pizza Done
	}
	else {
		vTimerSetTimerID( xTimer, (void * ) ulCount );
		vTaskResume( xTasks[BLUE_TIMER] );
	}
	
}

void vRedTimerCallback( TimerHandle_t xTimer) {
	uint32_t ulCount;
	
	ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
	ulCount++;
	
	if (ulCount >= RED_MAX_COUNT) {
		xTimerStop(xTimer, 0 );
		// Pizza Done
	}
	else {
		vTimerSetTimerID( xTimer, (void * ) ulCount );
		vTaskResume( xTasks[RED_TIMER] );
	}
	
}

void vGreenTimerCallback( TimerHandle_t xTimer) {
	uint32_t ulCount;
	
	ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
	ulCount++;
	
	if (ulCount >= GREEN_MAX_COUNT) {
		xTimerStop(xTimer, 0 );
		// Pizza Done
	}
	else {
		vTimerSetTimerID( xTimer, (void * ) ulCount );
		vTaskResume( xTasks[GREEN_TIMER] );
	}
	
}

void vOrangeTimerCallback( TimerHandle_t xTimer) {
	uint32_t ulCount;
	
	ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
	ulCount++;
	
	if (ulCount >= ORANGE_MAX_COUNT) {
		xTimerStop(xTimer, 0 );
		// Pizza Done
	}
	else {
		vTimerSetTimerID( xTimer, (void * ) ulCount );
		vTaskResume( xTasks[ORANGE_TIMER] );
	}
	
}



void vLedBlinkBlue(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_BLUE);
		vTaskDelay( BLINK_TIME / portTICK_RATE_MS );
		STM_EVAL_LEDToggle(LED_BLUE);
		vTaskSuspend( xTasks[BLUE_TIMER] );
		
	}
}

void vLedBlinkRed(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_RED);
		vTaskDelay( BLINK_TIME / portTICK_RATE_MS );
		STM_EVAL_LEDToggle(LED_RED);
		vTaskSuspend( xTasks[LED_RED] );
	}
}

void vLedBlinkGreen(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_GREEN);
		vTaskDelay( BLINK_TIME / portTICK_RATE_MS );
		STM_EVAL_LEDToggle(LED_GREEN);
		vTaskSuspend( xTasks[LED_GREEN] );
	}
}

void vLedBlinkOrange(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay( BLINK_TIME / portTICK_RATE_MS );
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskSuspend( xTasks[LED_ORANGE] );
	}
}

void vLongPressEvent(void *pvParameters) {
	vTimerSetTimerID(xTimers[4], (void *) 1);
}

// Taken from https://github.com/istarc/stm32/blob/master/examples/FreeRTOS/src/main.c
void detectButtonPress(void *pvParameters) {
		
	for(;;)
		{
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)>0) {
				
				// Button pressed
				while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) >0) {
					vTaskDelay((BOUNCE_THRESHOLD) / portTICK_RATE_MS); /* Button Debounce Delay */
					
					// Still pressed
					if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)>0) {
						if (!xTimerIsTimerActive(xTimers[4])) {
							vTimerSetTimerID(xTimers[4], 0);
							xTimerReset(xTimers[4], 0);
						}
						else if ( (int) pvTimerGetTimerID(xTimers[4]) == 1) {
							longPressEvent();
						}
					}
				}
				
				// Button lifted
				while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 0) {
					vTaskDelay((BOUNCE_THRESHOLD) / portTICK_RATE_MS); /* Button Debounce Delay */
					
					// Still lifted
					if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)>0) {
						if (xTimerIsTimerActive(xTimers[4])) {
							xTimerStop(xTimers[4], 0);
							shortPressEvent();
						}
					}
				}
		}
	}
}

void shortPressEvent() {
	
}

void longPressEvent() {
	
}

void initButton() 
{
	GPIO_InitTypeDef GPIO_InitStructure2;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure2.GPIO_Pin =  GPIO_Pin_0;
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure2);
}	
//******************************************************************************
