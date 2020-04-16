#include <stdio.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
//chu y 2 chia khoa giong nhau mo 2 cua giong nhau. thi phu thuoc thang nao mo truoc deu duoc
xSemaphoreHandle counting = 0;

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

int main(void)
{
    GPIO_Configuration();
    USART_Configuration(115200);	
    printf("-------FreeRTOS------\r\n");
    counting = xSemaphoreCreateCounting(2, 0); // chu y, tao ra 2 o khoa, dong lai ca 2
    xTaskCreate(TaskA, "TaskA", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskB, "TaskB", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskC, "TaskC", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(TaskD, "TaskD", 128, NULL, tskIDLE_PRIORITY, NULL);
    vTaskStartScheduler();
}

static void TaskA(void *pvParameters)
{  
    vTaskDelay(1000);
    xSemaphoreGive(counting);
    //neu them day 1 dong code nhu dong tren -> 2 den bat dau som tat cung luc
    for(;;) 
    {

    }
}


static void TaskB(void *pvParameters)
{  
    vTaskDelay(2000);
    xSemaphoreGive(counting);
    for(;;) 
    {

    }
}

static void TaskC(void *pvParameters)
{  
    xSemaphoreTake(counting, (TickType_t)0xFFFFFFFF); /*check o khoa dong hay mo > dung o day vi bi khoa
( sau khi het 1s tu task A -> mo cua o task A )
*/
    for(;;) 
    {
	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
	vTaskDelay(100);
    }
}

static void TaskD(void *pvParameters)
{  
    xSemaphoreTake(counting, (TickType_t)0xFFFFFFFF); /*dung o day cho nhu tren
sau do tuong tu giai thich nhu tren cho taskB*/
    for(;;) 
    {
	GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
	vTaskDelay(100);
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

