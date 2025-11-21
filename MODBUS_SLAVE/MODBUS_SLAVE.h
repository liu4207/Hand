#ifndef __MODBUS_SLAVE_H_
#define __MODBUS_SLAVE_H_

#include "main.h"
#include "gpio.h"
#include "my_system.h"
//LED¿ØÖÆ
typedef enum
{
  LED1 = (uint8_t)0x01,
  LED2 = (uint8_t)0x02,
}LED_NUM;


typedef struct
{
  void (*LED_ON)(uint8_t);
  void (*LED_OFF)(uint8_t);
  void (*LED_TOGGLE)(uint8_t);
}LED_t;

extern LED_t led;

//ÖÐ¶Ï¿ØÖÆ
#define uart_len 100

typedef struct
{
  uint8_t myaddr;//µØÖ·
  uint8_t modbus_buf[uart_len];//Êý¾Ý°ü
  uint16_t modbus_count;
  uint8_t rx_flag;//½ÓÊÜ±êÖ¾Î» ¿´¿´ÊÇ·ñ½ÓÊÕÊý¾Ý
  uint8_t timeout;//³¬Ê±Ê±¼ä 1sºÃÏñ
  uint8_t time_run;
  uint8_t send_buf[uart_len];
}MODBUS;

extern MODBUS modbus;

//UART´®¿ÚÉèÖÃ
extern UART_HandleTypeDef huart3;

void MY_UART_SendByte(uint8_t Byte);
void MY_UART_SendArray(uint8_t* Array,uint16_t len);
void MY_UART_SendString(char* String);

//RS485ÉèÖÃ Í¨¹ý¿ØÖÆËûµÄ¸ßµÍµçÆ½À´¿ØÖÆËûµÄ¶ÁÐ´ 
#define RS485_EN_W HAL_GPIO_WritePin(RS485_EN_GPIO_Port,RS485_EN_Pin,GPIO_PIN_SET)
#define RS485_EN_R HAL_GPIO_WritePin(RS485_EN_GPIO_Port,RS485_EN_Pin,GPIO_PIN_RESET)

void RS485_Send_Byte(uint8_t Byte);
void RS485_Send_Array(uint8_t* array,uint16_t len);

//MODBUS RTU
#define MODBUS_ADDR_LED1   0x0000
#define MODBUS_ADDR_LED2   0x0001


#define MB_SERVO_BASE        0x0100
#define MB_SERVO_ID          (MB_SERVO_BASE + 0)   // 0x0100:¶æ»úid
#define MB_GOAL_POS          (MB_SERVO_BASE + 1)   // 0x0101:Ä¿±êÎ»ÖÃ
#define MB_GOAL_SPEED        (MB_SERVO_BASE + 2)   // 0x0102:Ä¿±êËÙ¶È
#define MB_GOAL_TIME         (MB_SERVO_BASE + 3)   // 0x0103:ÔË¶¯Ê±¼ä
#define MB_TORQUE_ENABLE     (MB_SERVO_BASE + 4)   // 0x0104:Á¦¾ØÊ¹ÄÜ

#define MB_PRESENT_POS       (MB_SERVO_BASE + 5)   // 0x0105:¶ÁÈ¡Î»ÖÃ
#define MB_PRESENT_SPEED     (MB_SERVO_BASE + 6)
#define MB_PRESENT_LOAD      (MB_SERVO_BASE + 7)
#define MB_PRESENT_VOLT      (MB_SERVO_BASE + 8)
#define MB_PRESENT_TEMP      (MB_SERVO_BASE + 9)
#define MB_PRESENT_CURR      (MB_SERVO_BASE + 10)

#define MB_ACTION            (MB_SERVO_BASE + 11)  // Ð´1´¥·¢Ò»´Î¶¯×÷£¬ÔÙ×Ô¶¯ÇÇå
#define MB_PRESENT_ANGLE_DEG (MB_SERVO_BASE + 12)  // 

#define MB_DEBUG_LAST_STEPS  (MB_SERVO_BASE + 13)  // 0x010D:×î½üÒ»´ÎÏÂ·¢µÄ²½Êý
#define MB_DEBUG_FLAGS       (MB_SERVO_BASE + 14)  // 0x010E:µ÷ÊÔ±êÖ¾Î»

#define MB_LOOP_ENABLE     (MB_SERVO_BASE + 15)   // 0x010F:0=¹Ø,1=¿ª
#define MB_LOOP_ANGLE_A    (MB_SERVO_BASE + 16)   // 0x0110:A½Ç¶È
#define MB_LOOP_ANGLE_B    (MB_SERVO_BASE + 17)   // 0x0111:B½Ç¶È
#define MB_LOOP_DWELL_MS   (MB_SERVO_BASE + 18)   // 0x0112:µ½Î»Í£Áô(ms)
#define MB_LOOP_SPEED      (MB_SERVO_BASE + 19)   // 0x0113:Ñ­»·ËÙ¶È
#define MB_LOOP_TIME       (MB_SERVO_BASE + 20)   // 0x0114:Ñ­»·Ê±¼ä£¬ÓëËÙ¶È¶þÑ¡Ò»



typedef enum
{
  MODBUS_FUN_ERROR = 0x01,
  MODBUS_DATA_ADDR_ERROR = 0x02,
  MODBUS_DATA_ERROR = 0x03,
  MODBUS_DEVICE_ERROR = 0x04,  
}ERROR_t; //Òì³£Âë


void MODBUS_Init(void);
void MODBUS_EVENT_PRO(void);
void LED_TEST(void); //led1 led2:0

//MODBUS CRCÐ£Ñé
uint16_t crc16( uint8_t *puchMsg, uint16_t usDataLen );

#endif
