#include "myconfig.h"

void RS485_Send_Byte(uint8_t Byte)
{
  RS485_EN_W;
  MY_UART_SendByte(Byte);
  RS485_EN_R;
}

void RS485_Send_Array(uint8_t* array,uint16_t len)
{
  RS485_EN_W;
  MY_UART_SendArray(array,len);
  RS485_EN_R;
}

