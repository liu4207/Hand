#include "sensor_i2c.h"
#include <stdio.h> // 需要用到 printf
#include "i2c.h"   // 访问 hi2c2
#include "usart.h" // 访问 hlpuart1 (用于 printf 重定向)
#include "tim.h"   // 访问定时器回调

#include "MODBUS_SLAVE.h"
#include "scs_modbus_bridge.h"

/* Private Defines -----------------------------------------------------------*/
// I2C 传感器地址数组（7bit 地址）：10,11,12,13,14 -> 0x0A~0x0E
const uint8_t kSensorAddrs[5] = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E};

/* Private Variables ---------------------------------------------------------*/
// n 是用于 TIM3 中累加计数的局部变量



/* Private Function Prototypes -----------------------------------------------*/

/**
 * @brief  I2C Master 读取单个传感器的 XYZ 数据。
 * @param  addr7: 传感器 7-bit 地址。
 * @param  *x, *y, *z: 用于存储读取到的 XYZ 值的指针。
 * @retval HAL_StatusTypeDef: HAL_OK/HAL_ERROR
 */
static HAL_StatusTypeDef ReadXYZ_Once(uint8_t addr7, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t cmd = 0x03;
    uint8_t rx[6];

    // 1. 发送读取命令 0x03
    // 超时 20ms
    if (HAL_I2C_Master_Transmit(&hi2c2, addr7 << 1, &cmd, 1, 20) != HAL_OK) {
        return HAL_ERROR;
    }

    // 2. 接收 XYZ 数据 (6 字节)
    // 超时 20ms
    if (HAL_I2C_Master_Receive(&hi2c2, addr7 << 1, rx, 6, 20) != HAL_OK) {
        return HAL_ERROR;
    }

    // 3. 数据处理 (低位在前，转为 int16_t)
    *x = (int16_t)((rx[1] << 8) | rx[0]);
    *y = (int16_t)((rx[3] << 8) | rx[2]);
    *z = (int16_t)((rx[5] << 8) | rx[4]);

    return HAL_OK;
}


/* Exported Functions (I2C 传感器操作) --------------------------------------*/

void I2C_SetAddress(uint8_t current_addr7, uint8_t new_addr7)
{
    // 按传感器手册修正：仅发送 0x04 + 1字节新地址（7bit）
    uint8_t frame[2] = {0x04, new_addr7};
    HAL_I2C_Master_Transmit(&hi2c2, (current_addr7 << 1), frame, 2, HAL_MAX_DELAY);
}

void  ReceiveOneSensor(void)
{
    static uint8_t idx = 0; // 0..4，用于轮询 kSensorAddrs
    int16_t X, Y, Z;
    uint8_t a = kSensorAddrs[idx];

    if (ReadXYZ_Once(a, &X, &Y, &Z) == HAL_OK) {
        // 形如：A=0x0A, X=-12, Y=34, Z=567
        printf("A=0x%02X, X=%d, Y=%d, Z=%d\r\n", a, X, Y, Z);
    } else {
        // 可选：打印 I2C 错误信息，辅助调试
        // printf("I2C Error Code: 0x%08lX\r\n", hi2c2.ErrorCode); 
        printf("A=0x%02X, Read Fail\r\n", a);
    }
    // 切换到下一个传感器地址
    idx = (uint8_t)((idx + 1) % 5); 
}

void ReceiveAllSensors(void)
{
    for (size_t i = 0; i < 5; ++i) {
        int16_t X, Y, Z;
        uint8_t a = kSensorAddrs[i];
        if (ReadXYZ_Once(a, &X, &Y, &Z) == HAL_OK) {
            printf("A=0x%02X, X=%d, Y=%d, Z=%d\r\n", a, X, Y, Z);
        } else {
            printf("A=0x%02X, Read Fail\r\n", a);
        }
    }
}


/* Exported Functions (UART 重定向) ------------------------------------------*/

//串口重定义函数
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
// 实现 Keil/GNU GCC 环境下的 printf 函数
}


int _write(int file, char *ptr, int len) 
{
    // 阻塞发送，确保数据发送完整
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

/* Exported Functions (HAL 回调) ---------------------------------------------*/

/**
 * @brief  定时器周期溢出回调函数。
 * 负责维护 10ms 的节拍 tick10ms。
 * @param  htim: 定时器句柄。
 * @retval None
 */

    
//    // TIM2 的 LED 闪烁逻辑
//    if (htim->Instance == TIM2) { 
//        // 请确保 LED1_GPIO_Port, LED1_Pin 等宏已定义
//        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); // 切换 LED1
//    }
    
    // 注意：您原来的代码中 TIM3 的计数是 >=50，如果 TIM3 周期是 1ms，
    // 则 1ms * 50 = 50ms。这里我按您的注释“10ms”修改为 n >= 10。

		/* 正常读取一帧XYZ（6字节，低位在前），打印到USART1 */
//void Sensor_ReadOnce_Print(void){
//    uint8_t cmd = 0x03, rx[6];
//    if (HAL_I2C_Master_Transmit(&hi2c2, (0x0A<<1), &cmd, 1, 20) == HAL_OK &&
//        HAL_I2C_Master_Receive (&hi2c2, (0x0A<<1), rx, 6, 20) == HAL_OK){
//        int16_t X = (int16_t)((rx[1] << 8) | rx[0]);
//        int16_t Y = (int16_t)((rx[3] << 8) | rx[2]);
//        int16_t Z = (int16_t)((rx[5] << 8) | rx[4]);
//        printf("XYZ=%d,%d,%d\r\n", X, Y, Z);
//    }else{
//        printf("Read fail at 0x0A\r\n");
//    }
//}
