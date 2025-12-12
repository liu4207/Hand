#include "myconfig.h"

MODBUS modbus;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3) //1ms
  {
    if(modbus.time_run == 1)
    {
      modbus.timeout ++;
      if(modbus.timeout >= 7)
      {
        modbus.rx_flag = 1; //接收标志置为1，进行处理数据
        modbus.time_run = 0; //关闭定时器运行
      }      
    }
  HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);    
HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin); 
  }
  
//  if(htim->Instance == TIM2)
//  {
//    sensor.heart += 1;
//    sensor.humi += 2;
//    sensor.temp += 3;    
//  }  
}





void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {  
    if(modbus.rx_flag == 1) //处理数据并将数据发送给主机后，才会再一次接收数据
    {
      return ;      
    }    
    modbus.modbus_buf[modbus.modbus_count ++] = ch;
    modbus.timeout = 0;
    
    if(modbus.modbus_count == 1)
    {
      modbus.time_run = 1;
    }
    
    HAL_UART_Receive_IT(&huart2,&ch,1);
    
  }
}




