#include "myconfig.h"
#include "las10_bridge.h"

// 调试标志：控制电缸升降
volatile int test1 = 0;   // Keil里改成1触发
volatile uint16_t g_dbg_deg = 90;      // Keil里改角度
extern uint16_t MODBUS_Reg[];
extern LAS10_Dev g_devs[LAS_MAX_DEV];  // 外部声明
static void Per_Init(void);
static void System_Run(void);
uint8_t tx_buf[5] = {0x56,0x22,0x66,0x10,0xde};
char string[] = "guigui";
uint8_t ch;
System_t system = 
{
  Per_Init,
  System_Run,
};

static void Per_Init(void)
{
  MODBUS_Init();
}

static void System_Run(void)
{  
  MODBUS_EVENT_PRO();

  RS485_EN_R;
  //HAL_UART_Receive_IT(&huart2,&ch,1);

	if (test1)
		{
			//	MODBUS_Reg[0x0300]=4;
        test1 = 0;

        // 设置目标位置为调试角度
        MODBUS_Reg[MB_ADDR_LAS_GOAL_POS] = g_dbg_deg;   // 角度
			
		LAS10_Status st; // 电缸状态的结构体
    bool result = LAS10_SetGoalPos_Dev(&g_devs[4 - 1], MODBUS_Reg[MB_ADDR_LAS_GOAL_POS], &st, 20000);  // 调用设置目标位置的函数，ID 为 4
			
		}

}