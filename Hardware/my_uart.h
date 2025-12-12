#ifndef __MY_UART_H_
#define __MY_UART_H_


extern UART_HandleTypeDef huart2;

void MY_UART_SendByte(uint8_t Byte);
void MY_UART_SendArray(uint8_t* Array,uint16_t len);
void MY_UART_SendString(char* String);


#endif

