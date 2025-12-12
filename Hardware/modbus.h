#ifndef __MODBUS_H_
#define __MODBUS_H_



/* === 在这里追加 LAS 桥接使用的保持寄存器地址 === */
#define MB_ADDR_LAS_CMD           0x0000
#define MB_ADDR_LAS_GOAL_POS      0x0001
#define MB_ADDR_LAS_CUR_POS       0x0002
#define MB_ADDR_LAS_TEMP_C        0x0003
#define MB_ADDR_LAS_CURRENT_MA    0x0004
#define MB_ADDR_LAS_ERR_BITS      0x0005
#define MB_ADDR_LAS_FORCE_G       0x0006
#define MB_ADDR_LAS_GOAL_ECHO     0x0007
#define MB_ADDR_LAS_ID            0x0008
#define MB_ADDR_LAS_BAUD_CODE     0x0009
#define MB_ADDR_LAS_OCP_MA        0x000A
#define MB_ADDR_LAS_OTP_X10C      0x000B
#define MB_ADDR_LAS_RESUME_X10C   0x000C

/* ===== 多电缸块大小（一个电缸占用 0x0100 个地址） ===== */
#define LAS_MB_BLOCK_SIZE      0x0100   /* #1:0x0000~0x00FF, #2:0x0100~0x01FF, ... */

/* 总寄存器容量：4 块 + 预留 16 个作扩展/Group */
#define MODBUS_REG_SIZE     (4 * LAS_MB_BLOCK_SIZE + 0x25)
typedef enum 
{
  MODBUS_FUN_ERROR = 0x01,
  MODBUS_DATA_ADDR_ERROR = 0x02,
  MODBUS_DATA_ERROR = 0x03,
  MODBUS_DEVICE_ERROR = 0x04,  
}ERROR_t; //异常码

typedef struct
{
  uint16_t heart;
  uint16_t temp;
  uint16_t humi;  
}SENSOR;


void MODBUS_Init(void);
void MODBUS_EVENT_PRO(void);
void LED_TEST(void); //led1 led2:0
void SENSOR_TEST(void);

extern SENSOR sensor;

#endif

