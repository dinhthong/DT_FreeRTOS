#include <stdio.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static void Task_send(void *pvParameters);
static void Task_receive(void *pvParameters);
void GPIO_Configuration(void);
void USART_Configuration(unsigned int BaudRate);
void Delay(__IO uint32_t nCount);

xQueueHandle xQueue;

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
    xQueue = xQueueCreate( 10, sizeof( uint32_t ) );
    /*doi so dau tien: kich thuoc queue, toi da 10 messages
size: do bien minh truyen di la 4 bytes -> trong th nay tra ve so 4*/
    xTaskCreate(Task_send, "TaskA", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(Task_receive, "TaskB", 128, NULL, tskIDLE_PRIORITY, NULL);
    vTaskStartScheduler();
}
/*task chuyen send queue, vi du tinh toan thong so xong send. neu chung ta dung ham printf thay cho xQueueSend thi no se phu thuoc phan cung 
pl2303 -> mat thoi gian */
/* chia ra 1 task chuyen nhan de gui du lieu. Lam cho viec nay khong phu thuoc vao phan cung, 1 task chuyen nhan du lieu va printf ra ngoai man hinh
*/
static void Task_send(void *pvParameters)
{  
    uint32_t data_s = 0;
    for(;;) 
    {
        printf("Send %u to receiver task\n\r",data_s)
	    if(!xQueueSend(xQueue, (void*)&data_s, 1000) //doi so thu 3, timeout: so 0 co nghia neu message day thi bo qua luon
	    {
            printf("Failed to send to queue")
        }
    ++data_s;
	vTaskDelay(2000); // thay 1000 -> 1 > hien tuong nghen co chai > can thiet ke timeout.} vi du dat maximum time out 0xFFFFFFFFF.... 
	
    }
}

//task nhan queue va hien thi ra
static void Task_receive(void *pvParameters)
{ 
    uint32_t data_g=0;
    for(;;) 
    {
	if(xQueueReceive(xQueue, (void*)&data_g, 1000)
	{
	    printf("received:  %u\r\n", data_g);
	}
    else{
        printf("Failed to receive")
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

