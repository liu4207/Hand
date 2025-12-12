#include "MODBUS_SLAVE.h"
#include "scs_modbus_bridge.h"

static void Per_Init(void);
static void System_Run(void);

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
	  HAL_UART_Receive_IT(&huart2,&ch,1);

  
}

