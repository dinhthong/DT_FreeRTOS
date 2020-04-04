//******************************************************************************
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
//#include "discoveryf4utils.h"
//******************************************************************************
#define DEBUG 1
//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "semphr.h"
//******************************************************************************
static void TaskA(void *pvParameters);
static void TaskB(void *pvParameters);
static void TaskC(void *pvParameters);
static void TaskD(void *pvParameters);
static void TaskButton(void *pvParameters);
void GPIO_Configuration(void);
void usart_init(uint32_t BaudRate);
void EXTILine0_Config(void);

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/
xSemaphoreHandle mutex = 0;
xSemaphoreHandle xButtonSemaphore;
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
	//usart_init(115200);
	GPIO_Configuration();
	EXTILine0_Config();
		/* Create the semaphore used to wake the button handler task from the GPIO
	ISR. */
	vSemaphoreCreateBinary( xButtonSemaphore );
	xSemaphoreTake( xButtonSemaphore, 0 );
	
		printf("-------FreeRTOS------\r\n");
    mutex = xSemaphoreCreateMutex();
    xTaskCreate(TaskA, "TaskA", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskB, "TaskB", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskC, "TaskC", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskD, "TaskD", 128, NULL, tskIDLE_PRIORITY, NULL);
		xTaskCreate(TaskButton, "TaskButton", 528, NULL, configMAX_PRIORITIES - 1, NULL);
    vTaskStartScheduler();
}

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

void mutex_funtion(int x)
{
    if(mutex != NULL )
    {
	if( xSemaphoreTake(mutex, 0xFFFFFFFF) == pdTRUE) //ktra xem co the vao chua
	{
	    printf("1234567890 Day la resource accessed by %d\r\n",x);
	    xSemaphoreGive( mutex ); //unlock mutex
	}
	else
	{
	    
	}
    }
}

static void TaskA(void *pvParameters)
{  
    for(;;) 
    {
#if DEBUG
	mutex_funtion(1);
#else
	printf("1234567890 Day la TaskA\r\n");
#endif
    }
}


static void TaskB(void *pvParameters)
{ 
    for(;;) 
    {
#if DEBUG
	mutex_funtion(2);
#else
	printf("1234567890 Day la TaskB\r\n");
#endif
    }
}

static void TaskC(void *pvParameters)
{   
    for(;;) 
    {
#if DEBUG
	mutex_funtion(3);
#else
	printf("1234567890 Day la TaskC\r\n");
#endif
    }
}

static void TaskD(void *pvParameters)
{   
    for(;;) 
    {
//#if DEBUG
//	mutex_funtion(4);
//#else
//	printf("1234567890 Day la TaskD\r\n");
//#endif
			GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
			vTaskDelay(500);
    }
}

#define butDEBOUNCE_DELAY	( 200 / portTICK_RATE_MS )
static void TaskButton(void *pvParameters)
{
	/* Ensure the semaphore is created before it gets used. */
	//vSemaphoreCreateBinary( xButtonSemaphore );
	xSemaphoreTake( xButtonSemaphore, portMAX_DELAY);
	for( ;; )
	{
		/* Block on the semaphore to wait for an interrupt event.  The semaphore
		is 'given' from vButtonISRHandler() below.  Using portMAX_DELAY as the
		block time will cause the task to block indefinitely provided
		INCLUDE_vTaskSuspend is set to 1 in FreeRTOSConfig.h. */
		xSemaphoreTake( xButtonSemaphore, portMAX_DELAY);

		/* The button must have been pushed for this line to be executed.
		Simply toggle the LED. */
		//butLED1 = !butLED1;
		GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
		vTaskDelay(500);
		GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
//	if(mutex != NULL )
//    {
//	if( xSemaphoreTake(mutex, 0xFFFFFFFF) == pdTRUE) //ktra xem co the vao chua
//	{
//	    printf("1234567890 Day la resource accessed by ISR\r\n");
//	    xSemaphoreGive( mutex ); //unlock mutex
//	}
//	else
//	{
//	    
//	}
//}
		/* Wait a short time then clear any pending button pushes as a crude
		method of debouncing the switch.  xSemaphoreTake() uses a block time of
		zero this time so it returns immediately rather than waiting for the
		interrupt to occur. */
		//vTaskDelay( butDEBOUNCE_DELAY );
		//xSemaphoreTake( xButtonSemaphore, 0 );
	}
}

void EXTI0_IRQHandler(void)
{
	signed portBASE_TYPE sHigherPriorityTaskWoken = pdFALSE;
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
		EXTI_ClearITPendingBit(EXTI_Line0);
		xSemaphoreGiveFromISR( xButtonSemaphore, &sHigherPriorityTaskWoken );
		portYIELD_FROM_ISR(sHigherPriorityTaskWoken); 
    /* Clear the EXTI line 0 pending bit */
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

void EXTILine0_Config(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;
  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
