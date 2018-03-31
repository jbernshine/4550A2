#include "main.h"
#include "stm32f4xx_conf.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "codec.h"

#define STACK_SIZE_MIN 128
#define CLOCK_SPEED 84000000
#define PRESCALER 41999

#define MS_TO_FORLOOP_ITERATIONS 50000

//Get the period in ticks for a given period in miliseconds
int msToTicks(int periodInMs)
{
	return periodInMs * (CLOCK_SPEED / PRESCALER) / 1000 - 1;
}

typedef enum Boolean
{
	false,
	true,
} boolean;

//Tasks

typedef struct PIZZA
{
	int cookTime;
	uint16_t led;
	boolean cooking;
	float soundFrequency;
} Pizza;

typedef struct
{
	Pizza * pizza;
	char * taskName;
	TaskHandle_t taskHandle;
	TimerHandle_t timer;
	SemaphoreHandle_t semaphore;
} TaskData;

typedef enum PizzaType
{
	VEGGIE,
	HAWAIIAN,
	BBQ,
	DELUXE,
	NUM_PIZZAS,
} PizzaType;

Pizza pizzas[NUM_PIZZAS] = {
	[VEGGIE] = { 4, GPIO_Pin_13, false, 0.010,  },
	[HAWAIIAN] = { 6, GPIO_Pin_12, false, 0.016 },
	[BBQ] = { 8, GPIO_Pin_14, false, 0.017 },
	[DELUXE] = { 10, GPIO_Pin_15, false, 0.018 },
};

TaskData pizzaTasks[NUM_PIZZAS] = {
	[VEGGIE] = { .taskName = "Veggie" },
	[HAWAIIAN] = { .taskName = "Hawaiian" },
	[BBQ] = { .taskName = "BBW" },
	[DELUXE] = { .taskName = "Deluxe" },
};

//Times in miliseconds
#define BOUNCE_THRESHOLD 30
#define DOUBLE_PRESS_THRESHOLD 500
#define PIZZA_COUNTDOWN_PERIOD 1000
#define LONG_PRESS_THRESHOLD 300
#define SOUND_DURATION 500

#define BUTTON_TIMER TIM2

//File variables
volatile boolean isCooking;
volatile PizzaType selectedPizza = VEGGIE;

fir_8 filt;
SemaphoreHandle_t audioSemaphore;


//Hardware initiat

void InitLeds()
{
	GPIO_InitTypeDef GPIO_Initstructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
 	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_Initstructure);
}


void InitButton()
{
	GPIO_InitTypeDef GPIO_Initstructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_Initstructure);
}

void InitTimer()
{
	TIM_TimeBaseInitTypeDef timer_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	timer_InitStructure.TIM_Prescaler = PRESCALER; 
	timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timer_InitStructure.TIM_RepetitionCounter = 0;
	timer_InitStructure.TIM_Period = msToTicks(LONG_PRESS_THRESHOLD);
	TIM_TimeBaseInit(BUTTON_TIMER, &timer_InitStructure);
}

//Hardware control
void turnOffLed()
{
	GPIO_ResetBits(GPIOD, pizzas[selectedPizza].led);
}

void turnOnLed(uint16_t led)
{
	GPIO_ResetBits(GPIOD, 0xFFFF);
	GPIO_SetBits(GPIOD, led);
}

//Code adapted from sample project.
void playSound(float frequency)
{
	xSemaphoreTake(audioSemaphore, portMAX_DELAY);
	codec_ctrl_init();
	I2S_Cmd(CODEC_I2S, ENABLE);
	for (sampleCounter = 0; sampleCounter < SOUND_DURATION * 4000; sampleCounter++)
	{
	 	if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE))
   	{
   		SPI_I2S_SendData(CODEC_I2S, sample);
   		//only update on every second sample to insure that L & R ch. have the same sample value
   		if (sampleCounter & 0x00000001)
   		{
   			sawWave += frequency;
   			if (sawWave > 1.0)
   				sawWave -= 2.0;
    		filteredSaw = updateFilter(&filt, sawWave);
    		sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
    	}
    	sampleCounter++;
    }
	}
	codec_ctrl_init();
	xSemaphoreGive(audioSemaphore);
}

void startCooking()
{
	portBASE_TYPE pxTaskWoken;
	Pizza * pizza = &pizzas[selectedPizza];
	pizzas[selectedPizza].cooking = true;
	//All pizzas have same priority; hence, will never need to preempt here
	if (pxTaskWoken == pdTRUE)
	{
		portYIELD_FROM_ISR(pdTRUE);
	}
}

void stopCooking()
{
}

//Handle button input depending on the state of the program.
void shortPressEvent() {
	if (isCooking)
	{
		stopCooking();
	}
	else
	{
		startCooking();
	}
}

/* Getting button events
 * Use a software timer to measure the time taken
 * If below a certain threshold, it's a bounce -- ignore
 * Otherwse, its a press
 */


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
							vTimerSetTimerID(xButtonTimer, 0);
							xTimerReset(xButtonTimer, 0);
						}
						else if ( (int) pvTimerGetTimerID(xTimers[4] == 1) {
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

void debug()
{
	GPIO_SetBits(GPIOD, 0xFFFF);
}

SemaphoreHandle_t ovenSemaphore;

void pizzaTask(void * pvParameter)
{
	TaskData * taskData = (TaskData*) pvParameter;
	Pizza * pizza = taskData->pizza;
	for (;;)
	{
		if  (xSemaphoreTake(taskData->semaphore, portMAX_DELAY))
		{
			for (int i = 0; i < pizza->cookTime; i++)
			{
				if (xSemaphoreTake(ovenSemaphore, portMAX_DELAY))
				{
					GPIO_SetBits(GPIOD, pizza->led);
					vTaskDelay(500 / portTICK_RATE_MS);
					GPIO_ResetBits(GPIOD, pizza->led);
					vTaskDelay(500 / portTICK_RATE_MS);
					if (i == pizza->cookTime - 1)
					{
						playSound(pizza->soundFrequency);
					}
					xSemaphoreGive(ovenSemaphore);
				}
			}
			pizza->cooking = false;
			GPIO_ResetBits(GPIOD, pizza->led);
		}
	}
}

void createTasks()
{
	TaskData * taskData;
	for (int i = 0; i < NUM_PIZZAS; i++)
	{
		taskData = &pizzaTasks[i];
		taskData->pizza = &pizzas[i];
		taskData->taskHandle = (TaskHandle_t)i;
		xTaskCreate(pizzaTask, taskData->taskName, STACK_SIZE_MIN, (void*)taskData, 2, taskData->taskHandle);
		taskData->semaphore = xSemaphoreCreateBinary();
		if (taskData->semaphore == NULL)
			debug();
	}
}

int main()
{
	SystemInit();
	InitLeds();
	InitButton();
	InitTimer();
	ovenSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(ovenSemaphore);
	audioSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(audioSemaphore);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	codec_init();
	initFilter(&filt);
	
	createTasks();
	vTaskStartScheduler();
}

// a very crude FIR lowpass filter
float updateFilter(fir_8* filt, float val)
{
	uint16_t valIndex;
	uint16_t paramIndex;
	float outval = 0.0;

	valIndex = filt->currIndex;
	filt->tabs[valIndex] = val;

	for (paramIndex=0; paramIndex<8; paramIndex++)
	{
		outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
	}

	valIndex++;
	valIndex &= 0x07;

	filt->currIndex = valIndex;
	portTICK_RATE_MS;
	return outval;
}

void initFilter(fir_8* theFilter)
{
	uint8_t i;

	theFilter->currIndex = 0;

	for (i=0; i<8; i++)
		theFilter->tabs[i] = 0.0;

	theFilter->params[0] = 0.01;
	theFilter->params[1] = 0.05;
	theFilter->params[2] = 0.12;
	theFilter->params[3] = 0.32;
	theFilter->params[4] = 0.32;
	theFilter->params[5] = 0.12;
	theFilter->params[6] = 0.05;
	theFilter->params[7] = 0.01;
}