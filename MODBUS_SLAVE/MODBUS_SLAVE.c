#include "MODBUS_SLAVE.h"
#include "scs_modbus_bridge.h"


//中断设置
MODBUS modbus;

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  if(htim->Instance == TIM3) //1ms
//  {
//    if(modbus.time_run == 1)
//    {
//      modbus.timeout ++;
//      if(modbus.timeout >= 7)
//      {
//        modbus.rx_flag = 1; //接收标志置为1，进行处理数据
//        modbus.time_run = 0; //关闭定时器运行
//      }      
//    }
//			 HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
//       HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);    
//  }
//}


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

//UART串口设置....................  ...........................................................................................................................................

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

//RS485设置...........................................................................................................................................................................
extern UART_HandleTypeDef huart2;

static inline void wait_tx_tc(void){
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {;}
}

void RS485_Send_Byte(uint8_t Byte)
{
                           
  MY_UART_SendByte(Byte);                
  //wait_tx_tc();                          
                         
}

void RS485_Send_Array(uint8_t* array,uint16_t len)
{
 
  MY_UART_SendArray(array,len);          
 // wait_tx_tc();                          

}

//MODBUS RTU设置....................................................................................................................................................
#define MODBUS_REG_CAP 0x0200   
uint16_t MODBUS_Reg[MODBUS_REG_CAP] = {0};


void MODBUS_Init(void)
{
  modbus.myaddr = 1;
  modbus.time_run = 0;
 SCS_Bridge_Init();
	MODBUS_Reg[MB_SERVO_ID]   = 1;
	MODBUS_Reg[MB_SERVO_ID2] =  2;  // 设置舵机 2 （ID2）
	MODBUS_Reg[MB_SERVO_ID3]  = 3;  // 设置舵机 3 的 ID 为 3
    MODBUS_Reg[MB_SERVO_ID4]  = 4;  // 设置舵机 4 的 ID 为 4
    MODBUS_Reg[MB_SERVO_ID5]  = 5;  // 设置舵机 5 的 ID 为 5
    MODBUS_Reg[MB_SERVO_ID6]  = 6;  // 设置舵机 6 的 ID 为 6
    MODBUS_Reg[MB_SERVO_ID7]  = 7;  // 设置舵机 7 的 ID 为 7
    MODBUS_Reg[MB_SERVO_ID8]  = 8;  // 设置舵机 8 的 ID 为 8
    MODBUS_Reg[MB_GOAL_SPEED] = 1500;
	MODBUS_Reg[MB_GOAL_SPEED2] = 1500;

  MODBUS_Reg[MB_GOAL_TIME]  = 0;
	MODBUS_Reg[MB_GOAL_TIME2]  = 0;
}

void LED_TEST(void) //led1 led2:0
{
  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,(GPIO_PinState)MODBUS_Reg[MODBUS_ADDR_LED1]);
  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,(GPIO_PinState)MODBUS_Reg[MODBUS_ADDR_LED2]);
}


void MODBUS_ERROR(uint8_t fun,ERROR_t error)
{
  uint16_t i = 0;
  uint16_t j;  
  uint16_t crc;

  modbus.send_buf[i++] = modbus.myaddr;
  modbus.send_buf[i++] = 0x80 + fun;
  modbus.send_buf[i++] = error;
  
  crc = crc16(modbus.send_buf, i);
  modbus.send_buf[i++] = crc%256;
  modbus.send_buf[i++] = crc/256;
  for(j=0;j<i;j++)
  {
    RS485_Send_Byte(modbus.send_buf[j]);
  }  
  
}

void MODBUS_FUNCTION_03(void)
{
  uint16_t i = 0;
  uint16_t j;
  uint16_t ADDR_STAR;
  uint16_t ADDR_LEN;
  uint8_t LEN;
  uint16_t crc;
  
  modbus.send_buf[i++] = modbus.myaddr;
  modbus.send_buf[i++] = 0x03;
  
  ADDR_STAR = modbus.modbus_buf[2]*256 + modbus.modbus_buf[3]; // (modbus.modbus_buf[2]) << 8 | modbus.modbus_buf[3] 256 = 2^8   
  ADDR_LEN = modbus.modbus_buf[4]*256 + modbus.modbus_buf[5]; 
  LEN = 2*ADDR_LEN;
  modbus.send_buf[i++] = LEN;

	if (ADDR_STAR + ADDR_LEN > MODBUS_REG_CAP)
		{
    MODBUS_ERROR(0x03, MODBUS_DATA_ADDR_ERROR);
    return;
		}
  
  for(j=0;j<ADDR_LEN;j++)
  {
    modbus.send_buf[i++] = MODBUS_Reg[ADDR_STAR + j]/256;
    modbus.send_buf[i++] = MODBUS_Reg[ADDR_STAR + j]%256;      
  }
  crc = crc16(modbus.send_buf, i);
  modbus.send_buf[i++] = crc%256;
  modbus.send_buf[i++] = crc/256;
  for(j=0;j<i;j++)
  {
    RS485_Send_Byte(modbus.send_buf[j]);
  }  
}



void MODBUS_FUNCTION_06(void)
{
  uint16_t i = 0;
  uint16_t j;
  uint16_t ADDR_STAR;  
  uint16_t Data;
  uint16_t crc;

  
  modbus.send_buf[i++] = modbus.myaddr; //01
  modbus.send_buf[i++] = 0x06; 
  
  ADDR_STAR = modbus.modbus_buf[2]*256 + modbus.modbus_buf[3]; // (modbus.modbus_buf[2]) << 8 | modbus.modbus_buf[3] 256 = 2^8   
  Data = modbus.modbus_buf[4]*256 + modbus.modbus_buf[5]; 

  MODBUS_Reg[ADDR_STAR] = Data;
  SCS_Bridge_OnWrite(ADDR_STAR, 1);
  modbus.send_buf[i++] = modbus.modbus_buf[2];
  modbus.send_buf[i++] = modbus.modbus_buf[3];
  modbus.send_buf[i++] = modbus.modbus_buf[4];
  modbus.send_buf[i++] = modbus.modbus_buf[5]; 
  
  crc = crc16(modbus.send_buf, i);
  modbus.send_buf[i++] = crc%256;
  modbus.send_buf[i++] = crc/256;
  
  if (ADDR_STAR >= MODBUS_REG_CAP) 
	  {
    MODBUS_ERROR(0x06, MODBUS_DATA_ADDR_ERROR);
    return;
	}
	  
  for(j=0;j<i;j++)
  {
    RS485_Send_Byte(modbus.send_buf[j]);
  }    
}


void MODBUS_FUNCTION_10(void)
{
  uint16_t i = 0;
  uint16_t j;
  uint16_t ADDR_STAR;
  uint16_t ADDR_LEN;
  uint8_t Byte;
  uint16_t crc;
    
  ADDR_STAR = modbus.modbus_buf[2]*256 + modbus.modbus_buf[3]; // (modbus.modbus_buf[2]) << 8 | modbus.modbus_buf[3] 256 = 2^8   
  ADDR_LEN = modbus.modbus_buf[4]*256 + modbus.modbus_buf[5]; 
  Byte = modbus.modbus_buf[6];
  
  if(Byte != ADDR_LEN*2)
  {
    return ;
  }
  if (ADDR_STAR + ADDR_LEN > MODBUS_REG_CAP)
	  {
    MODBUS_ERROR(0x10, MODBUS_DATA_ADDR_ERROR);
    return;
		}
  
  for(j=0;j<ADDR_LEN;j++)
  {
    MODBUS_Reg[ADDR_STAR + j] = modbus.modbus_buf[7 + 2*j]*256 + modbus.modbus_buf[7 + 2*j + 1];    
  }
  
  
   SCS_Bridge_OnWrite(ADDR_STAR, ADDR_LEN);
  
  modbus.send_buf[i++] = modbus.myaddr; //01
  modbus.send_buf[i++] = 0x10;    
  modbus.send_buf[i++] = modbus.modbus_buf[2];
  modbus.send_buf[i++] = modbus.modbus_buf[3];  
  modbus.send_buf[i++] = modbus.modbus_buf[4];
  modbus.send_buf[i++] = modbus.modbus_buf[5];  
  
  crc = crc16(modbus.send_buf, i);
  modbus.send_buf[i++] = crc%256;
  modbus.send_buf[i++] = crc/256;
  for(j=0;j<i;j++)
  {
    RS485_Send_Byte(modbus.send_buf[j]);
  }      


}



// 03:查询 06：更改单个寄存器 10：一次性更改多个寄存器
//CRC校验是否正确
void MODBUS_EVENT_PRO(void)
{
 
  if(modbus.rx_flag == 0)
  {
    return ;
  }

  
  if(modbus.modbus_buf[0] == modbus.myaddr)
  {
    uint8_t lo = modbus.modbus_buf[modbus.modbus_count - 2];
  uint8_t hi = modbus.modbus_buf[modbus.modbus_count - 1];

  uint16_t rx_lo_first  = (uint16_t)(lo | (hi << 8));   // ???(??RTU)
  uint16_t rx_hi_first  = (uint16_t)((lo << 8) | hi);   // ???(??/????)
  uint16_t calc         = crc16(modbus.modbus_buf, modbus.modbus_count - 2);

      if (calc == rx_lo_first || calc == rx_hi_first)
    {
      switch(modbus.modbus_buf[1])
      {
        case 03:MODBUS_FUNCTION_03();break;
        case 06:MODBUS_FUNCTION_06();break;
        case 16:MODBUS_FUNCTION_10();break;        //10十六进制 16十进制
        case 01:case 02:MODBUS_ERROR(modbus.modbus_buf[1],MODBUS_FUN_ERROR);break;
      }           
    }
  }
  modbus.rx_flag = 0;
  modbus.modbus_count = 0;
}

//MODBUS CRC校验
const uint8_t auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;

const uint8_t auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;


/******************************************************************

******************************************************************/
uint16_t crc16( uint8_t *puchMsg, uint16_t usDataLen )
{
    uint8_t uchCRCHi = 0xFF ; 
    uint8_t uchCRCLo = 0xFF ;
    unsigned long uIndex ; 		

    while ( usDataLen-- ) 	
    {
        uIndex = uchCRCHi ^ *puchMsg++ ; 	
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
        uchCRCLo = auchCRCLo[uIndex] ;
    }

    return ( uchCRCHi << 8 | uchCRCLo ) ;
}
