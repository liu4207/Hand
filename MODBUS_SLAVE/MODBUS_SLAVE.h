#ifndef __MODBUS_SLAVE_H_
#define __MODBUS_SLAVE_H_

#include "main.h"
#include "gpio.h"
#include "my_system.h"

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
extern UART_HandleTypeDef huart2;

void MY_UART_SendByte(uint8_t Byte);
void MY_UART_SendArray(uint8_t* Array,uint16_t len);
void MY_UART_SendString(char* String);


void RS485_Send_Byte(uint8_t Byte);
void RS485_Send_Array(uint8_t* array,uint16_t len);

//MODBUS RTU
#define MODBUS_ADDR_LED1   0x0500
#define MODBUS_ADDR_LED2   0x0501
// ========== ¼Ä´æÆ÷³ØÈÝÁ¿£¨Îñ±Ø¸²¸Çµ½ >= 0x0800+ÄãµÄ×î´óÆ«ÒÆ£©==========
// 0x1000 = 4096 regs -> 8KB£¬STM32H7ÍêÈ«¹»ÓÃ
#define MODBUS_REG_SIZE    0x1000

// ========== SCS µ¥Ìå¼Ä´æÆ÷Çø£¨°á¼Ò£¬±ÜÃâºÍµç¸×block³åÍ»£©==========
#define MB_SERVO_BASE      0x0800
//#define MB_SERVO_ID          (MB_SERVO_BASE + 0)   // 0x0100:¶æ»úid
//#define MB_GOAL_POS          (MB_SERVO_BASE + 1)   // 0x0101:Ä¿±êÎ»ÖÃ
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

//ÐÂµÄ¶æ»ú id==2
// ¶æ»ú2¿ØÖÆ¼Ä´æÆ÷
//#define MB_SERVO_ID2         (MB_SERVO_BASE + 50)  // 0x0132:¶æ»ú2 ID
//#define MB_GOAL_POS2         (MB_SERVO_BASE + 51)  // 0x0133:¶æ»ú2 Ä¿±êÎ»ÖÃ
#define MB_GOAL_SPEED2       (MB_SERVO_BASE + 52)  // 0x0134:¶æ»ú2 Ä¿±êËÙ¶È
#define MB_GOAL_TIME2        (MB_SERVO_BASE + 53)  // 0x0135:¶æ»ú2 ÔË¶¯Ê±¼ä
#define MB_TORQUE_ENABLE2    (MB_SERVO_BASE + 54)  // 0x0136:¶æ»ú2 Á¦¾ØÊ¹ÄÜ
#define MB_PRESENT_POS2      (MB_SERVO_BASE + 55)  // 0x0137:¶æ»ú2 ¶ÁÈ¡Î»ÖÃ
#define MB_PRESENT_SPEED2    (MB_SERVO_BASE + 56)  // 0x0138:¶æ»ú2 ¶ÁÈ¡ËÙ¶È
#define MB_PRESENT_LOAD2     (MB_SERVO_BASE + 57)  // 0x0139:¶æ»ú2 ¶ÁÈ¡¸ºÔØ
#define MB_PRESENT_VOLT2     (MB_SERVO_BASE + 58)  // 0x013A:¶æ»ú2 ¶ÁÈ¡µçÑ¹
#define MB_PRESENT_TEMP2     (MB_SERVO_BASE + 59)  // 0x013B:¶æ»ú2 ¶ÁÈ¡ÎÂ¶È
#define MB_PRESENT_CURR2     (MB_SERVO_BASE + 60)  // 0x013C:¶æ»ú2 ¶ÁÈ¡µçÁ÷
#define MB_ACTION2           (MB_SERVO_BASE + 61)  // 0x013D:¶æ»ú2 Ð´1´¥·¢¶¯×÷
#define MB_PRESENT_ANGLE_DEG2 (MB_SERVO_BASE + 62) // 0x013E:¶æ»ú2 µ±Ç°½Ç¶È£¨¶È£©
#define MB_DEBUG_LAST_STEPS2 (MB_SERVO_BASE + 63)  // 0x013F:¶æ»ú2 ×î½üÒ»´ÎÏÂ·¢µÄ²½Êý
#define MB_DEBUG_FLAGS2      (MB_SERVO_BASE + 64)  // 0x0140:¶æ»ú2 µ÷ÊÔ±êÖ¾Î»
#define MB_LOOP_ENABLE2      (MB_SERVO_BASE + 65)  // 0x0141:¶æ»ú2 Ñ­»·ÆôÓÃ£¨0=¹Ø±Õ, 1=¿ªÆô£©
#define MB_LOOP_ANGLE_A2     (MB_SERVO_BASE + 66)  // 0x0142:¶æ»ú2 A ½Ç¶È
#define MB_LOOP_ANGLE_B2     (MB_SERVO_BASE + 67)  // 0x0143:¶æ»ú2 B ½Ç¶È
#define MB_LOOP_DWELL_MS2    (MB_SERVO_BASE + 68)  // 0x0144:¶æ»ú2 µ½Î»Í£ÁôÊ±¼ä£¨ms£©
#define MB_LOOP_SPEED2       (MB_SERVO_BASE + 69)  // 0x0145:¶æ»ú2 Ñ­»·ËÙ¶È
#define MB_LOOP_TIME2        (MB_SERVO_BASE + 70)  // 0x0146:¶æ»ú2 Ñ­»·Ê±¼ä£¨ÓëËÙ¶È¶þÑ¡Ò»£©

//#define MB_GOAL_POS    0x0302   // 0x0151: Ä¿±êÎ»ÖÃ£¨ID1£©
//#define MB_GOAL_POS2   0x0302  // 0x0152: ¶æ»ú 2 Ä¿±êÎ»ÖÃ
//#define MB_GOAL_POS3   (MB_SERVO_BASE + 83)  // 0x0153: ¶æ»ú 3 Ä¿±êÎ»ÖÃ
//#define MB_GOAL_POS4   (MB_SERVO_BASE + 84)  // 0x0154: ¶æ»ú 4 Ä¿±êÎ»ÖÃ
//#define MB_GOAL_POS5   (MB_SERVO_BASE + 85)  // 0x0155: ¶æ»ú 5 Ä¿±êÎ»ÖÃ
//#define MB_GOAL_POS6   (MB_SERVO_BASE + 86)  // 0x0156: ¶æ»ú 6 Ä¿±êÎ»ÖÃ
//#define MB_GOAL_POS7   (MB_SERVO_BASE + 87)  // 0x0157: ¶æ»ú 7 Ä¿±êÎ»ÖÃ
//#define MB_GOAL_POS8   (MB_SERVO_BASE + 88)  // 0x0158: ¶æ»ú 8 Ä¿±êÎ»ÖÃ

#define MB_SERVO_ID          (MB_SERVO_BASE + 90)   // 0x0100:¶æ»úid
#define MB_SERVO_ID2         (MB_SERVO_BASE + 91)  // 0x0132:¶æ»ú2 ID

 
#define MB_SERVO_ID3         (MB_SERVO_BASE + 92)   // 0x0133: ¶æ»ú 3 ID
#define MB_SERVO_ID4         (MB_SERVO_BASE + 93)   // 0x0134: ¶æ»ú 4 ID
#define MB_SERVO_ID5         (MB_SERVO_BASE + 94)   // 0x0135: ¶æ»ú 5 ID
#define MB_SERVO_ID6         (MB_SERVO_BASE + 95)   // 0x0136: ¶æ»ú 6 ID
#define MB_SERVO_ID7         (MB_SERVO_BASE + 96)   // 0x0137: ¶æ»ú 7 ID
#define MB_SERVO_ID8         (MB_SERVO_BASE + 97)   // 0x0138: ¶æ»ú 8 ID


#define MB_GOAL_POS    (MB_SERVO_BASE + 81)  // 0x0151: Ä¿±êÎ»ÖÃ£¨ID1£©
#define MB_GOAL_POS2   (MB_SERVO_BASE + 82)  // 0x0152: ¶æ»ú 2 Ä¿±êÎ»ÖÃ
#define MB_GOAL_POS3   (MB_SERVO_BASE + 83)  // 0x0153: ¶æ»ú 3 Ä¿±êÎ»ÖÃ
#define MB_GOAL_POS4   (MB_SERVO_BASE + 84)  // 0x0154: ¶æ»ú 4 Ä¿±êÎ»ÖÃ
#define MB_GOAL_POS5   (MB_SERVO_BASE + 85)  // 0x0155: ¶æ»ú 5 Ä¿±êÎ»ÖÃ
#define MB_GOAL_POS6   (MB_SERVO_BASE + 86)  // 0x0156: ¶æ»ú 6 Ä¿±êÎ»ÖÃ
#define MB_GOAL_POS7   (MB_SERVO_BASE + 87)  // 0x0157: ¶æ»ú 7 Ä¿±êÎ»ÖÃ
#define MB_GOAL_POS8   (MB_SERVO_BASE + 88)  // 0x0158: ¶æ»ú 8 Ä¿±êÎ»ÖÃ


/* === ÔÚÕâÀï×·¼Ó LAS ÇÅ½ÓÊ¹ÓÃµÄ±£³Ö¼Ä´æÆ÷µØÖ· === */
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

/* ===== ¶àµç¸×¿é´óÐ¡£¨Ò»¸öµç¸×Õ¼ÓÃ 0x0100 ¸öµØÖ·£© ===== */
#define LAS_MB_BLOCK_SIZE      0x0100   /* #1:0x0000~0x00FF, #2:0x0100~0x01FF, ... */


//Êµ¼ÊÒªÓÃµÄ

/* ---- Group ²Ù×÷¼Ä´æÆ÷£¨È«¾Ö¶Î£»²»ÓëËÄ¸ö¿é³åÍ»£©---- */
#define MB_GRP_MASK   0x0400  /* Ñ¡ÔñÄÄÐ©µç¸×£ºbit0=#1, bit1=#2, bit2=#3, bit3=#4£¨1=Ñ¡ÖÐ£© */
#define MB_GRP_CMD    0x0401  /* Ð´µ¥¿ØÖ¸Áî£º0x04¹¤×÷/0x23¼±Í£/0x14ÔÝÍ£/0x1EÇå¹ÊÕÏ/0x22²éÑ¯... */
#define MB_GRP_GOAL   0x0402  /* Ð´Ä¿±êÎ»ÖÃ£¨0~2000£©£º¶ÔÑ¡ÖÐµç¸×Ò»ÆðÏÂ·¢ */

/* ---- LAS10Group ¶àÄ¿±ê£¨Ò»´ÎÐ´ËÄ¸ö£©+ Ìá½» ---- */
#define MB_GRP_MPOS1  0x0410  /* #1 Ä¿±êÎ»£¨0~2000£© */
#define MB_GRP_MPOS2  0x0411  /* #2 Ä¿±êÎ» */
#define MB_GRP_MPOS3  0x0412  /* #3 Ä¿±êÎ» */
#define MB_GRP_MPOS4  0x0413  /* #4 Ä¿±êÎ» */
#define MB_GRP_APPLY  0x0414  /* Ð´ 1 ´¥·¢£º°Ñ 0410~0413 ÉÈ³öµ½ #1~#4 */

// ========== SCS Group£¨ÐÂÔö£º½ô½Ó LAS Group£¬±ãÓÚÒ»Ìõ0x10¸²¸Ç£©==========
#define MB_SCS_MASK        0x0415

#define MB_SCS_GPOS1       0x0416
#define MB_SCS_GPOS2       0x0417
#define MB_SCS_GPOS3       0x0418
#define MB_SCS_GPOS4       0x0419
#define MB_SCS_GPOS5       0x041A
#define MB_SCS_GPOS6       0x041B
#define MB_SCS_GPOS7       0x041C

#define MB_SCS_GSPD        0x041D
#define MB_SCS_GTIME       0x041E
#define MB_SCS_APPLY       0x041F

// MODBUS_SLAVE.h ÐÂÔö
#define MB_FB_BASE      0x0420
#define MB_FB_LAS_POS1  (MB_FB_BASE + 0) // 0x0420

#define MB_FB_SCS_ANG1  (MB_FB_BASE + 4) // 0x0424

#define MB_FB_SCS_STEPS1  0x0430

typedef enum
{
  MODBUS_FUN_ERROR = 0x01,
  MODBUS_DATA_ADDR_ERROR = 0x02,
  MODBUS_DATA_ERROR = 0x03,
  MODBUS_DEVICE_ERROR = 0x04,  
}ERROR_t; //Òì³£Âë

typedef struct
{
  uint16_t heart;
  uint16_t temp;
  uint16_t humi;  
}SENSOR;

void MODBUS_Init(void);
void MODBUS_EVENT_PRO(void);
void LED_TEST(void); //led1 led2:0
extern SENSOR sensor;
//MODBUS CRCÐ£Ñé
uint16_t crc16( uint8_t *puchMsg, uint16_t usDataLen );

#endif
