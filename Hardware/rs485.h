#ifndef __RS485_H_
#define __RS485_H_

#define RS485_EN_W HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET)
#define RS485_EN_R HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET)

void RS485_Send_Byte(uint8_t Byte);
void RS485_Send_Array(uint8_t* array,uint16_t len);

#endif

