#include "myconfig.h"

uint16_t MODBUS_Reg[MODBUS_REG_SIZE];
SENSOR sensor;

void MODBUS_Init(void)
{
  modbus.myaddr = 1;
  modbus.time_run = 0;
  /* 同一条 UART2 带多只 D 型电缸：分别给块基址与ID */
  LASM_ClearAll();
  LASM_Add(0, &huart1, 1, 0x0000, NULL, 0);  // 电缸#1 → ID=1  → 0x0000~0x00FF
  LASM_Add(1, &huart1, 2, 0x0100, NULL, 0);  // 电缸#2 → ID=2  → 0x0100~0x01FF
  // 继续加：
  LASM_Add(2, &huart1, 3, 0x0200, NULL, 0);
  LASM_Add(3, &huart1, 4, 0x0300, NULL, 0);
	MODBUS_Reg[MB_GRP_MASK] = 0x000F;
 
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
	
LASM_OnReadMaybeRefresh(ADDR_STAR, ADDR_LEN); // ← 新：涉及到的电缸先刷新状态

  
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
LASM_OnWriteSingle(ADDR_STAR, Data);   // ← 新：多设备桥接
  
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
  
  for(j=0;j<ADDR_LEN;j++)
  {
    MODBUS_Reg[ADDR_STAR + j] = modbus.modbus_buf[7 + 2*j]*256 + modbus.modbus_buf[7 + 2*j + 1];    
  }
LASM_OnWriteBlock(ADDR_STAR, &MODBUS_Reg[ADDR_STAR], ADDR_LEN); // ← 新

  
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




