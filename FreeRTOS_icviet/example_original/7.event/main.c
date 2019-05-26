#include <stdio.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

EventGroupHandle_t event_hdl;

#define EVENT0		(1<<0)
#define EVENT1		(1<<1)
#define EVENT2		(1<<2)
#define EVENT_ALLS	((EVENT2<<1) - 1)

static void TaskMain(void *pvParameters);
static void TaskA(void *pvParameters);
static void TaskB(void *pvParameters);
static void TaskC(void *pvParameters);
static void TaskD(void *pvParameters);
void GPIO_Configuration(void);
void USART_Configuration(unsigned int BaudRate);
void Delay(__IO uint32_t nCount);

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
/*bat phat event, mot thang phat event, co mot hoac vai thang dung bat event  ( de lam mot viec gi do tuy ung dung )
*/
int main(void)
{
    GPIO_Configuration();
    USART_Configuration(115200);	
    printf("-------FreeRTOS------\r\n");
    xTaskCreate(TaskMain, "TaskMain", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskA, "TaskA", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskB, "TaskB", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskC, "TaskC", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskD, "TaskD", 128, NULL, tskIDLE_PRIORITY, NULL);
    vTaskStartScheduler();
}

static void TaskMain(void *pvParameters)
{  
  //create event.
    event_hdl = xEventGroupCreate();
    vTaskDelay(1000);
    //sau 1s, event dau tien phat di la EVENT0
    xEventGroupSetBits(event_hdl, EVENT0);
    vTaskDelay(1000);
    xEventGroupSetBits(event_hdl, EVENT1);
    vTaskDelay(1000);
    xEventGroupSetBits(event_hdl, EVENT2);
    vTaskDelay(1000);
    for(;;) 
    {
	
    }
}
//cac task ABCD la cac ham bat su kien
static void TaskA(void *pvParameters)
{  
    EventBits_t event;
    for(;;) 
    {
      //bat event, sau khi bat event thi togglebit de lam dau hieu bat duoc event
	event = xEventGroupWaitBits(event_hdl, EVENT_ALLS, pdTRUE, pdFALSE, portMAX_DELAY);
	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
        //bat duoc EVENT0 -> cho pin_13 high
	if (event & EVENT0)
	{
	    GPIO_SetBits(GPIOD, GPIO_Pin_13);
	}
	else if (event & EVENT1)
	{
	    GPIO_SetBits(GPIOD, GPIO_Pin_14);
	}
	else if (event & EVENT2)
	{
	    GPIO_SetBits(GPIOD, GPIO_Pin_15);
	}
    }
}

//task B chi bat 1 EVENT0 thoi
static void TaskB(void *pvParameters)
{ 
    EventBits_t event;
    for(;;) 
    {
	event = xEventGroupWaitBits(event_hdl, EVENT_ALLS, pdTRUE, pdFALSE, portMAX_DELAY);
	/*printf ra ->>>> do TaskA lam togglebit bat EVENT0 va TaskB printf ra khi bat EVENT0 nen ta se thay 2 nhiem vu nay xay ra dong thoi
        */
	if (event & EVENT0)
	{
	    printf("%s | event:%d\r\n", __FUNCTION__, event);
	}
    }
}

static void TaskC(void *pvParameters)
{   
    EventBits_t event;
    for(;;) 
    {
	event = xEventGroupWaitBits(event_hdl, EVENT_ALLS, pdTRUE, pdFALSE, portMAX_DELAY);
	
	if (event & EVENT1)
	{
	    printf("%s | event:%d\r\n", __FUNCTION__, event);
	}
    }
}

static void TaskD(void *pvParameters)
{   
    EventBits_t event;
    for(;;) 
    {
	event = xEventGroupWaitBits(event_hdl, EVENT_ALLS, pdTRUE, pdFALSE, portMAX_DELAY);
	
	if (event & EVENT2)
	{
	    printf("%s | event:%d\r\n", __FUNCTION__, event);
	}
    }
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

void USART_Configuration(unsigned int BaudRate)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = BaudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); 
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); 
    
    USART_Cmd(USART1, ENABLE);  
}



void Delay(__IO uint32_t nCount)
{
    while(nCount--)
    {
    }
}

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART_SendData(USART1, (uint8_t) ch);
    
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}
    
    return ch;
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    
    /* Infinite loop */
    while (1)
    {
    }
}
#endif

