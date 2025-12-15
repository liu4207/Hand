#include "MODBUS_SLAVE.h"
#include "scs_modbus_bridge.h"
#include "las10_bridge.h"

extern void SCS_ExecWritePosN(uint16_t reg_id, uint16_t reg_pos,
                               uint16_t reg_time, uint16_t reg_speed);
static void Per_Init(void);
static void System_Run(void);
uint8_t tx_buf[5] = {0x56,0x22,0x66,0x10,0xde};
char string[] = "guigui";


extern uint16_t MODBUS_Reg[];


uint8_t ch;
System_t system = 
{
  Per_Init,
  System_Run,
};


static void Per_Init(void)
{
  MODBUS_Init();
  SCS_Bridge_Init();
}


static void System_Run(void)
{  
	  MODBUS_EVENT_PRO();	
	  LED_TEST();
	  SCS_Bridge_Poll_20ms(); 
	LASM_Poll_50ms();
	  HAL_UART_Receive_IT(&huart2,&ch,1);
		
		
  
}

