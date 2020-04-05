#include <stdio.h>
#include <time.h>
#include <rt_misc.h>
#include "stm32f4xx.h"

#pragma import(__use_no_semihosting_swi)

//USART2
//#define USARTx USART2//EVAL_COM2
extern UART_HandleTypeDef huart2;
extern long timeval;        

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

//When use printf() -> it will call function 'fputc' N times.
int fputc(int ch, FILE *f) {
  /*This bit is set by hardware when the content of the TDR (USART_DR: data register when in Transfer mode) register has been transferred into
the shift register. */
 while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == RESET){}
 // HAL_USART_SendData(USARTx, ch);
	 HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return(ch);
  /* default
  return (sendchar(ch));
  */
}

int fgetc(FILE *f) {
uint8_t ch;
//when the content of .... 
    while(__HAL_UART_GET_FLAG(&huart2, USART_FLAG_RXNE) == RESET){}
    HAL_UART_Receive(&huart2, &ch, 1, 0);
    return ch;
  /*//default
  return (sendchar(getkey()));
  */
}

int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}

void _ttywrch(int ch) {
  while(__HAL_UART_GET_FLAG(&huart2, USART_FLAG_TXE) == RESET){}
  //USART_SendData(USARTx, ch);
		HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
}

void _sys_exit(int return_code) {
  while (1);    /* endless loop */
}
#define netconn_write(conn, dataptr, size, apiflags) \ netconn_write_partly(conn, dataptr, size, apiflags, NULL)
