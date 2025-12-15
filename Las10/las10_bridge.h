#ifndef __LAS10_BRIDGE_H__
#define __LAS10_BRIDGE_H__

#include "main.h"
#include "usart.h"     // 需要 huart1
#include <stdint.h>
#include <stdbool.h>

/* ---------------- Modbus 地址映射（可按需扩展） ---------------- */
/* 写寄存器 */
#define MB_ADDR_LAS_CMD           0x0000  /* 单控指令值（直接写入 0x04/0x23/0x14/0x20/0x22/0x1E 等） */
#define MB_ADDR_LAS_GOAL_POS      0x0001  /* 目标位置 0~2000 */ //目前是0x0001 0x0101 0x0201 0x0301

/* 读寄存器（状态） */
#define MB_ADDR_LAS_CUR_POS       0x0002  /* 当前位置（有符号 16） */
#define MB_ADDR_LAS_TEMP_C        0x0003  /* 温度（°C，int16） */
#define MB_ADDR_LAS_CURRENT_MA    0x0004  /* 电流（mA，uint16） */
#define MB_ADDR_LAS_ERR_BITS      0x0005  /* 错误信息 bit0堵转、bit1过温、bit2过流、bit3电机异常 */
#define MB_ADDR_LAS_FORCE_G       0x0006  /* 力传感器（g，int16，非力控型无意义） */
#define MB_ADDR_LAS_GOAL_ECHO     0x0007  /* 最近一次下发的目标位置（回显） */

/* 写寄存器（参数） */
#define MB_ADDR_LAS_ID            0x0008  /* 电缸ID（1~254），写入后需“参数装订”并重启才固化 */
#define MB_ADDR_LAS_BAUD_CODE     0x0009  /* 波特率代码 0:19200 1:57600 2:115200 3:921600，装订+掉电生效 */
#define MB_ADDR_LAS_OCP_MA        0x000A  /* 过流保护设定 mA，范围约 300~1500 */
#define MB_ADDR_LAS_OTP_X10C      0x000B  /* 过温保护 温度*10 */
#define MB_ADDR_LAS_RESUME_X10C   0x000C  /* 回温启动 温度*10 */

/* ---------------- LAS10 协议常量 ---------------- */
#define LAS_HDR1                  0x55
#define LAS_HDR2                  0xAA
#define LAS_RSP_HDR1              0xAA
#define LAS_RSP_HDR2              0x55

/* 指令号 */
#define LAS_CMD_RD                0x01
#define LAS_CMD_WR                0x02
#define LAS_CMD_MOVE_FB           0x21  /* 定位模式（反馈状态） */
#define LAS_CMD_MOVE_NFB          0x03  /* 定位模式（无反馈） */
#define LAS_CMD_FOLLOW_FB         0x20  /* 随动（反馈） */
#define LAS_CMD_FOLLOW_NFB        0x19  /* 随动（无反馈） */
#define LAS_CMD_MC                0x04  /* 单控指令 */

/* 控制表索引（仅对 WR/Move 等有效） */
#define LAS_IDX_GOAL_POS          0x37  /* 目标位置起始地址（2B, low->high） */

/* 单控指令参数2（按手册） */
#define LAS_MC_WORK               0x04  /* 工作/上电使能 */
#define LAS_MC_ESTOP              0x23  /* 急停（禁止功率输出）*/
#define LAS_MC_PAUSE              0x14  /* 暂停 */
#define LAS_MC_BIND               0x20  /* 参数装订（烧写到Flash） */
#define LAS_MC_QUERY              0x22  /* 查询状态信息 */
#define LAS_MC_CLR_ERR            0x1E  /* 清故障 */

/* ---- Group 操作寄存器（全局段；不与四个块冲突）---- */
#define MB_GRP_MASK   0x0400  /* 选择哪些电缸：bit0=#1, bit1=#2, bit2=#3, bit3=#4（1=选中） */
#define MB_GRP_CMD    0x0401  /* 写单控指令：0x04工作/0x23急停/0x14暂停/0x1E清故障/0x22查询... */
#define MB_GRP_GOAL   0x0402  /* 写目标位置（0~2000）：对选中电缸一起下发 */

/* ---- Group 多目标（一次写四个）+ 提交 ---- */
#define MB_GRP_MPOS1  0x0410  /* #1 目标位（0~2000） */
#define MB_GRP_MPOS2  0x0411  /* #2 目标位 */
#define MB_GRP_MPOS3  0x0412  /* #3 目标位 */
#define MB_GRP_MPOS4  0x0413  /* #4 目标位 */
#define MB_GRP_APPLY  0x0414  /* 写 1 触发：把 0410~0413 扇出到 #1~#4 */


/* 结构体：状态 */
typedef struct {
    uint16_t goal_pos;     /* 目标位置 */
    int16_t  cur_pos;      /* 当前位置 */
    int16_t  temp_c;       /* 温度（°C） */
    uint16_t current_mA;   /* 电流（mA） */
    int16_t  force_g;      /* 力（g，非力控型无意义） */
    uint8_t  err_bits;     /* 错误信息 bits */
} LAS10_Status;

/* 初始化：设置电缸ID（默认1） */
void LAS10_Init(uint8_t las_id);

/* 执行：单控指令（param2 取上面的 LAS_MC_XX）并可选解析状态 */
bool LAS10_SendMC(uint8_t param2, LAS10_Status* out_status, uint32_t timeout_us);

/* 执行：设置目标位置（0~2000），使用 带反馈 的定位指令 */
bool LAS10_SetGoalPos(uint16_t pos, LAS10_Status* out_status, uint32_t timeout_us);

/* 查询：状态（单控指令 0x22） */
bool LAS10_QueryStatus(LAS10_Status* out_status, uint32_t timeout_us);

/* 参数写：ID、波特率、过流、过温、回温 —— 写控制表（需装订+上电生效的注明） */
bool LAS10_WriteID(uint8_t new_id);
bool LAS10_WriteBaudCode(uint8_t code_0_3);
bool LAS10_WriteOCP(uint16_t mA);
bool LAS10_WriteOTP_x10C(uint16_t x10C);
bool LAS10_WriteResume_x10C(uint16_t x10C);

/* --------- 供 Modbus 桥接层调用的轻量封装 --------- */
void LAS10_Bridge_OnWriteSingle(uint16_t addr, uint16_t value);
void LAS10_Bridge_OnWriteBlock(uint16_t addr_start, const uint16_t* data, uint16_t len);
void LAS10_Bridge_OnReadMaybeRefresh(uint16_t addr_start, uint16_t qty);

#endif

/* ====== 多电缸支持（在文件末尾追加） ====== */
#ifndef LAS10_MULTI_H
#define LAS10_MULTI_H

#define LAS_MAX_DEV            4     /* 需要更多就加大 */
#define LAS_MB_BLOCK_SIZE      0x0100

typedef void (*LAS_SelectFn)(uint8_t slot, bool enable); /* 单UART复用时可用；并挂总线则为NULL */

/* 一个电缸实例 */
typedef struct {
    UART_HandleTypeDef* huart;   /* 串口句柄（本方案多个电缸都用 &huart2） */
    uint8_t  id;                 /* 电缸ID（D型默认1） */
    uint16_t mb_base;            /* Modbus块基址，如0x0000/0x0100/... */
    LAS_SelectFn select_fn;      /* 复用器选通（并挂总线= NULL） */
    uint8_t  select_slot;        /* 复用器通道（并挂总线=0） */
    LAS10_Status cache;          /* 最近一次查询到的状态 */
} LAS10_Dev;

/* 设备表管理 */
void LASM_ClearAll(void);
bool LASM_Add(uint8_t idx, UART_HandleTypeDef* huart, uint8_t id,
              uint16_t mb_base, LAS_SelectFn sel, uint8_t slot);

/* 通过 Modbus 地址定位设备与块内偏移 */
LAS10_Dev* LASM_FindByAddr(uint16_t mb_addr, uint16_t* offset);

/* 面向“设备”的 API */
bool LAS10_SendMC_Dev(LAS10_Dev* dev, uint8_t param2, LAS10_Status* out, uint32_t timeout_us);
bool LAS10_SetGoalPos_Dev(LAS10_Dev* dev, uint16_t pos, LAS10_Status* out, uint32_t timeout_us);
bool LAS10_QueryStatus_Dev(LAS10_Dev* dev, LAS10_Status* out, uint32_t timeout_us);

/* 多设备 Modbus 钩子（替换单设备版） */
void LASM_OnWriteSingle(uint16_t mb_addr, uint16_t value);
void LASM_OnWriteBlock(uint16_t mb_addr_start, const uint16_t* data, uint16_t len);
void LASM_OnReadMaybeRefresh(uint16_t mb_addr_start, uint16_t qty);

void LASM_Poll_50ms(void);
#endif /* LAS10_MULTI_H */
