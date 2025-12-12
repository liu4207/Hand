#include "myconfig.h"

void MY_UART_SendByte(uint8_t Byte)
{
  HAL_UART_Transmit(&huart2,&Byte,1,1000);
}

void MY_UART_SendArray(uint8_t* Array,uint16_t len)
{
  HAL_UART_Transmit(&huart2,Array,len,1000);  
}

void MY_UART_SendString(char* String)
{
  uint8_t i;
  for(i=0;String[i]!='\0';i++)
  {
    MY_UART_SendByte(String[i]);
  }
}
