#ifndef __SENSOR_I2C_H
#define __SENSOR_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// I2C 传感器地址数组（7bit 地址）
extern const uint8_t kSensorAddrs[5];

// 外部引用（在 main.c 中定义，在 sensor_i2c.c 中使用）
extern uint8_t tick10ms;


/**
 * @brief  初始化 I2C 传感器的地址。
 * @param  current_addr7: 当前地址 (7-bit)
 * @param  new_addr7: 新地址 (7-bit)
 * @retval None
 */
void I2C_SetAddress(uint8_t current_addr7, uint8_t new_addr7);


/**
 * @brief  读取单个 I2C 传感器（XYZ 6字节），并打印到串口。
 * 轮询 kSensorAddrs 数组中的下一个传感器。
 * @retval None
 */
void  ReceiveOneSensor(void);


/**
 * @brief  读取所有 5 个 I2C 传感器（XYZ 6字节），并打印到串口。
 * @retval None
 */
void ReceiveAllSensors_Print(void);


#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_I2C_H */
