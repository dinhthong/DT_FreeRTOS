//******************************************************************************
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
//#include "discoveryf4utils.h"
//******************************************************************************

//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
//******************************************************************************
static void TaskA(void *pvParameters);
static void TaskB(void *pvParameters);
static void TaskC(void *pvParameters);
static void TaskD(void *pvParameters);
void GPIO_Configuration(void);
void usart_init(uint32_t BaudRate);
//void vLedBlinkBlue(void *pvParameters);
//void vLedBlinkRed(void *pvParameters);
//void vLedBlinkGreen(void *pvParameters);
//void vLedBlinkOrange(void *pvParameters);

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

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
	usart_init(115200);
	GPIO_Configuration();
//	STM_EVAL_LEDInit(LED_BLUE);
//	STM_EVAL_LEDInit(LED_GREEN);
//	STM_EVAL_LEDInit(LED_ORANGE);
//	STM_EVAL_LEDInit(LED_RED);
	
    xTaskCreate(TaskA, (const signed char*)"TaskA", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskB, (const signed char*)"TaskB", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskC, (const signed char*)"TaskC", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskD, (const signed char*)"TaskD", 128, NULL, tskIDLE_PRIORITY, NULL);
	
	  vTaskStartScheduler();
}
//******************************************************************************

//******************************************************************************
//void vLedBlinkBlue(void *pvParameters)
//{
//	for(;;)
//	{
//		STM_EVAL_LEDToggle(LED_BLUE);
//		vTaskDelay( 500 / portTICK_RATE_MS );
//	}
//}

//void vLedBlinkRed(void *pvParameters)
//{
//	for(;;)
//	{
//		STM_EVAL_LEDToggle(LED_RED);
//		vTaskDelay( 750 / portTICK_RATE_MS );
//	}
//}

//void vLedBlinkGreen(void *pvParameters)
//{
//	for(;;)
//	{
//		STM_EVAL_LEDToggle(LED_GREEN);
//		vTaskDelay( 250 / portTICK_RATE_MS );
//	}
//}

//void vLedBlinkOrange(void *pvParameters)
//{
//	for(;;)
//	{
//		STM_EVAL_LEDToggle(LED_ORANGE);
//		vTaskDelay( 900 / portTICK_RATE_MS );
//	}
//}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOA, ENABLE);
    
    /* Configure PB0 PB1 in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    /* Configure PA0 in input mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
static void TaskA(void *pvParameters) //NULL
{  
    for(;;) 
    {
	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
	vTaskDelay(1500); //block state -> cac task khac co the xu ly tiep
    }
}


static void TaskB(void *pvParameters)
{ 
    for(;;) 
    {
	GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
	vTaskDelay(250);
    }
}

static void TaskC(void *pvParameters)
{   
    for(;;) 
    {
	GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
	vTaskDelay(100);
    }
}

static void TaskD(void *pvParameters)
{   
    for(;;) 
    {
			printf("Hello world\r\n");
	GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
	vTaskDelay(1500);
    }
}
void usart_init(uint32_t BaudRate) {
	GPIO_InitTypeDef GPIO_InitStruct;
USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef  NVIC_InitStruct;
//GPIO_InitTypeDef  GPIO_InitStruct;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);//UART Tx pin
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);//UART Rx pin

    USART_InitStruct.USART_BaudRate=BaudRate;
    USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
    USART_InitStruct.USART_Parity=USART_Parity_No;
    USART_InitStruct.USART_StopBits=USART_StopBits_1;
    USART_InitStruct.USART_WordLength=USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 10;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}
//******************************************************************************
