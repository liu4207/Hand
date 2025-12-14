#include "MODBUS_SLAVE.h"
#include "scs_modbus_bridge.h"

extern void SCS_ExecWritePosN(uint16_t reg_id, uint16_t reg_pos,
                               uint16_t reg_time, uint16_t reg_speed);
static void Per_Init(void);
static void System_Run(void);

//debug
volatile int test1 = 0;   // Keil里改成1触发
volatile uint16_t g_dbg_deg = 90;      // Keil里改角度
//
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
	
//		if (test1) {
//        test1 = 0;

//        MODBUS_Reg[MB_GOAL_POS]   = g_dbg_deg;   // 角度
//		
//		        // 直接执行“角度->步数->WritePos”
//        SCS_ExecWritePosN(MB_SERVO_ID, MB_GOAL_POS, MB_GOAL_TIME, MB_GOAL_SPEED);
//    }
	
	  HAL_UART_Receive_IT(&huart2,&ch,1);
		
		
  
}

