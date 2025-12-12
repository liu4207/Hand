// scs_modbus_bridge.c  ——modbus与SCS2332桥接层
// 角度与步数换算　即写即动 默认速度兜底


#include "stm32g4xx_hal.h"
#include "SCS.h"
#include "SCSCL.h"
#include "MODBUS_SLAVE.h"
#include <stdbool.h>
#include <stdint.h>

extern void setEnd(uint8_t v);
extern uint16_t MODBUS_Reg[];



static inline uint8_t sid(void)
{
    uint16_t id = MODBUS_Reg[MB_SERVO_ID];// 获取舵机的 ID（通过 MODBUS 寄存器中的 `MB_SERVO_ID`）

    if (id == 0) id = 1;          
    return (uint8_t)id;
}
// ====A<->B ====
typedef struct {
    uint8_t  phase;            // 0 idle, 1 moving, 2 dwell
    uint8_t  toB;              // 0 -> A;1 -> B
    uint16_t tgt_deg;          // 当前目标角度
    uint32_t t0;               // 停留计时七起点
} loop_ctx_t;

static loop_ctx_t s_loop;
#define LOOP_TOL_DEG  3        // 到位容差

// 复用 WritePos ：把角度发给舵机
static void SCS_GoAngleDeg(uint16_t deg, uint16_t spd, uint16_t tim){ //舵机的最大角度为 270°，与舵机的 1024 步数对应。
	
		uint8_t id = sid();  // 获取舵机的 ID（通过 MODBUS 寄存器中的 `MB_SERVO_ID`）

    if (id == 0) id = 1;  // 如果没有设置 ID，默认使用 ID 1
	
    if (deg > 360) deg = (uint16_t)((deg<<8)|(deg>>8));   // 防上位机发反
    if (deg > 270) deg = 270;
    // 兜底速度: speed=0 time=0 
    if (spd == 0 && tim == 0) spd = 1500; //若速度（spd）和时间（tim）为零，则设定一个默认的速度（1500）。速度和时间会传递到 WritePos 函数，控制舵机的运动。

    // 角度到步数： round(deg * 1024 / 270)
    uint32_t t = (uint32_t)deg * 1024u + 135u;
    uint16_t steps = (uint16_t)(t / 270u);
    if (steps > 1024) steps = 1024;

    MODBUS_Reg[MB_DEBUG_LAST_STEPS] = steps;    //调试回写 本质上决定了舵机运动多少步数！ 
    MODBUS_Reg[MB_DEBUG_FLAGS]     |= 0x01;

   // WritePos((uint8_t)MODBUS_Reg[MB_SERVO_ID], steps, tim, spd); 
    WritePos(id, steps, tim, spd);  
	  s_loop.tgt_deg = deg;

}


static void SCS_LoopStartNow(void){ //该函数启动一个往返循环，即舵机从角度 A 移动到 B，之后再返回到 A，形成一个往返运动。
      // 取循环参数MB_GOAL_SPEED/TIME)
    uint16_t spd = MODBUS_Reg[MB_LOOP_SPEED] ? MODBUS_Reg[MB_LOOP_SPEED] : MODBUS_Reg[MB_GOAL_SPEED];
    uint16_t tim = MODBUS_Reg[MB_LOOP_TIME]  ? MODBUS_Reg[MB_LOOP_TIME]  : MODBUS_Reg[MB_GOAL_TIME];
    uint16_t a   = MODBUS_Reg[MB_LOOP_ANGLE_A];
      //先去A 
    s_loop.phase = 1; s_loop.toB = 1;
    SCS_GoAngleDeg(a, spd, tim);
}

	//角度到步数

static uint16_t angle_deg_to_steps(uint16_t angle_raw){
    uint16_t angle = angle_raw;
    
    if (angle > 360) {
        angle = (uint16_t)((angle << 8) | (angle >> 8));
    }
    if (angle > 270) angle = 270;     // SCS2332 的范围0-270
    // steps = round(angle * 1024 / 270)
    uint32_t t = (uint32_t)angle * 1024u + 135u; // +270/2 ?????
    uint16_t steps = (uint16_t)(t / 270u);
    return (steps > 1024) ? 1024 : steps;
}

// 通道化执行：把“角度寄存器”换算为步数后写入对应舵机
static void SCS_ExecWritePosN(uint16_t reg_id, uint16_t reg_pos,
                              uint16_t reg_time, uint16_t reg_speed)
{
    uint8_t  id    = (uint8_t)MODBUS_Reg[reg_id];
    if (id == 0) id = 1;

    uint16_t deg   = MODBUS_Reg[reg_pos];
    uint16_t steps = angle_deg_to_steps(deg);   // ★统一换算角度→步数

    uint16_t time  = MODBUS_Reg[reg_time];
    uint16_t speed = MODBUS_Reg[reg_speed];
    if (time == 0 && speed == 0) speed = 1500;  // ★兜底速度

    MODBUS_Reg[MB_DEBUG_LAST_STEPS] = steps;
    MODBUS_Reg[MB_DEBUG_FLAGS]     |= 0x01;

    WritePos(id, steps, time, speed);
}




void SCS_Bridge_Init(void)
	{
    setEnd(1);                      
//		MODBUS_Reg[MB_SERVO_ID] = 2;   // 设置舵机 1 （ID1）
//    MODBUS_Reg[MB_SERVO_ID2] = 2;  // 设置舵机 2 （ID2）
   if (MODBUS_Reg[MB_SERVO_ID] == 0) MODBUS_Reg[MB_SERVO_ID] = 1;
    MODBUS_Reg[MB_TORQUE_ENABLE] = 1;//默认使能舵机 开始
		MODBUS_Reg[MB_TORQUE_ENABLE2] = 1;
    EnableTorque((uint8_t)MODBUS_Reg[MB_SERVO_ID], 1);
		EnableTorque((uint8_t)MODBUS_Reg[MB_SERVO_ID2], 1);
	}


void SCS_Bridge_OnWrite(uint16_t startAddr, uint16_t len)
{
    bool t1=false,t2=false,t3=false,t4=false,t5=false,t6=false,t7=false;
    uint16_t end = startAddr + len;

    for (uint16_t a = startAddr; a < end; ++a){
        // 扭矩
        if (a == MB_TORQUE_ENABLE){
            EnableTorque((uint8_t)MODBUS_Reg[MB_SERVO_ID],
                         (uint8_t)MODBUS_Reg[MB_TORQUE_ENABLE]);
            continue;
        }

        // === 即写即动触发源：任一被写，就标记对应通道 ===
        if (a==MB_GOAL_POS || a==MB_GOAL_SPEED || a==MB_GOAL_TIME) t1=true;
        if (a==MB_GOAL_POS2|| a==MB_GOAL_SPEED|| a==MB_GOAL_TIME) t2=true;
        if (a==MB_GOAL_POS3|| a==MB_GOAL_SPEED|| a==MB_GOAL_TIME) t3=true;
        if (a==MB_GOAL_POS4|| a==MB_GOAL_SPEED|| a==MB_GOAL_TIME) t4=true;
        if (a==MB_GOAL_POS5|| a==MB_GOAL_SPEED|| a==MB_GOAL_TIME) t5=true;
        if (a==MB_GOAL_POS6|| a==MB_GOAL_SPEED|| a==MB_GOAL_TIME) t6=true;
        if (a==MB_GOAL_POS7|| a==MB_GOAL_SPEED|| a==MB_GOAL_TIME) t7=true;

        // ACTION=1 兼容：默认作用到通道1（如需多通道ACTION可自行扩展）
        if (a == MB_ACTION && MODBUS_Reg[MB_ACTION]){
            t1 = true;
            MODBUS_Reg[MB_DEBUG_FLAGS] |= 0x02;
            MODBUS_Reg[MB_ACTION] = 0;
        }

        // 你的 LOOP 参数处理保持原样（如有）
        if (a >= MB_LOOP_ENABLE && a <= MB_LOOP_TIME){
            if (MODBUS_Reg[MB_LOOP_ENABLE]) {
                SCS_LoopStartNow();
            } else {
                s_loop.phase = 0;
            }
        }
    }

    // === 统一在这里执行（角度→步数→WritePos），每一路都正确 ===
    if (t1) SCS_ExecWritePosN(MB_SERVO_ID, MB_GOAL_POS, MB_GOAL_TIME, MB_GOAL_SPEED);
    if (t2) SCS_ExecWritePosN(MB_SERVO_ID2,MB_GOAL_POS2,MB_GOAL_TIME2,MB_GOAL_SPEED2);
    if (t3) SCS_ExecWritePosN(MB_SERVO_ID3,MB_GOAL_POS3,MB_GOAL_TIME,MB_GOAL_SPEED);
    if (t4) SCS_ExecWritePosN(MB_SERVO_ID4,MB_GOAL_POS4,MB_GOAL_TIME,MB_GOAL_SPEED);
    if (t5) SCS_ExecWritePosN(MB_SERVO_ID5,MB_GOAL_POS5,MB_GOAL_TIME,MB_GOAL_SPEED);
    if (t6) SCS_ExecWritePosN(MB_SERVO_ID6,MB_GOAL_POS6,MB_GOAL_TIME,MB_GOAL_SPEED);
    if (t7) SCS_ExecWritePosN(MB_SERVO_ID7,MB_GOAL_POS7,MB_GOAL_TIME,MB_GOAL_SPEED);
}

//每隔20ms主循环跑一次

void SCS_Bridge_Poll_20ms(void){
    static uint32_t tick = 0;
    if (HAL_GetTick() - tick < 20) return;
    tick = HAL_GetTick();

	   // 读取第一个舵机
    uint8_t id = sid();
    int v;

      // 位置步数——回写
    v = ReadPos(id);
    if (v >= 0){
        MODBUS_Reg[MB_PRESENT_POS] = (uint16_t)v;

         // 步数 -> 角度():deg = round(steps * 270 / 1024)
        uint16_t steps = (uint16_t)v;
        uint32_t t = (uint32_t)steps * 270u + 512u; // +512 实现 /1024 四舍五入
        uint16_t deg = (uint16_t)(t >> 10);         // 除以1024
        if (deg > 270) deg = 270;
        MODBUS_Reg[MB_PRESENT_ANGLE_DEG] = deg;     // 0x010C
    }
		
		  // 读取第二个舵机
    uint8_t id2 = 2;  // 第二个舵机的 ID
    v = ReadPos(id2);
    if (v >= 0) {
        MODBUS_Reg[MB_PRESENT_POS2] = (uint16_t)v;

        // 步数 -> 角度
        uint16_t steps = (uint16_t)v;
        uint32_t t = (uint32_t)steps * 270u + 512u;
        uint16_t deg = (uint16_t)(t >> 10);  
        if (deg > 270) deg = 270;
        MODBUS_Reg[MB_PRESENT_ANGLE_DEG2] = deg;  // 更新第二个舵机的角度
    }
		
		
	    // [LOOP PATCH] 循环状态A <-> B
    if (MODBUS_Reg[MB_LOOP_ENABLE]){
        uint16_t dwell = MODBUS_Reg[MB_LOOP_DWELL_MS];
        if (dwell == 0) dwell = 300;   // ?? 300ms ??
        uint16_t spd = MODBUS_Reg[MB_LOOP_SPEED] ? MODBUS_Reg[MB_LOOP_SPEED] : MODBUS_Reg[MB_GOAL_SPEED];
        uint16_t tim = MODBUS_Reg[MB_LOOP_TIME]  ? MODBUS_Reg[MB_LOOP_TIME]  : MODBUS_Reg[MB_GOAL_TIME];
        uint16_t a   = MODBUS_Reg[MB_LOOP_ANGLE_A];
        uint16_t b   = MODBUS_Reg[MB_LOOP_ANGLE_B];

        if (s_loop.phase == 0){
              // 未启动：按当前配置立刻启动
            SCS_LoopStartNow();
        } else if (s_loop.phase == 1){
           // 运动中：到位判断
            uint16_t cur = MODBUS_Reg[MB_PRESENT_ANGLE_DEG];
            uint16_t tgt = s_loop.tgt_deg;
            uint16_t diff = (cur > tgt) ? (cur - tgt) : (tgt - cur);
            if (diff <= LOOP_TOL_DEG){
                s_loop.phase = 2;
                s_loop.t0 = HAL_GetTick();
            }
        } else if (s_loop.phase == 2){
             // 停留结束，切换目标
            if (HAL_GetTick() - s_loop.t0 >= dwell){
                s_loop.phase = 1;
                if (s_loop.toB){
                    s_loop.toB = 0;  SCS_GoAngleDeg(b, spd, tim);
                } else {
                    s_loop.toB = 1;  SCS_GoAngleDeg(a, spd, tim);
                }
            }
        }
    } else {
        s_loop.phase = 0;   // 关闭状态机复位
    }


    v = ReadSpeed(id);   if (v >= 0) MODBUS_Reg[MB_PRESENT_SPEED] = (uint16_t)v;
    v = ReadLoad(id);    if (v >= 0) MODBUS_Reg[MB_PRESENT_LOAD]  = (uint16_t)v;
    v = ReadVoltage(id); if (v >= 0) MODBUS_Reg[MB_PRESENT_VOLT]  = (uint16_t)v;
    v = ReadTemper(id);  if (v >= 0) MODBUS_Reg[MB_PRESENT_TEMP]  = (uint16_t)v;
    v = ReadCurrent(id); if (v >= 0) MODBUS_Reg[MB_PRESENT_CURR]  = (uint16_t)(v & 0xFFFF);
}
