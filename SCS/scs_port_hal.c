#include "stm32g4xx_hal.h"
#include "usart.h"
#include <stdint.h>

// UART2 _URT-1
extern UART_HandleTypeDef huart4;

void ftUart_Send(uint8_t *nDat, int nLen){
    HAL_UART_Transmit(&huart4, nDat, nLen, 1000);
}

int ftUart_Read(uint8_t *nDat, int nLen){
    if (HAL_UART_Receive(&huart4, nDat, nLen, 20) == HAL_OK) return nLen;
    return 0;
}

void ftBus_Delay(void){
    HAL_Delay(1);
}
