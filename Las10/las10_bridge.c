#include "las10_bridge.h"
#include <string.h>
#include "MODBUS_SLAVE.h"
/* ===== 设备表 ===== */
LAS10_Dev g_devs[LAS_MAX_DEV];
static uint8_t   g_dev_count = 0;
static uint16_t g_grp_pos[LAS_MAX_DEV] = {0,0,0,0};

void LASM_ClearAll(void) {
    memset(g_devs, 0, sizeof(g_devs));
    g_dev_count = 0;
}

bool LASM_Add(uint8_t idx, UART_HandleTypeDef* huart, uint8_t id,
              uint16_t mb_base, LAS_SelectFn sel, uint8_t slot)
{
    if (idx >= LAS_MAX_DEV) return false;
    g_devs[idx].huart = huart;
    g_devs[idx].id = id;
    g_devs[idx].mb_base = mb_base;
    g_devs[idx].select_fn = sel;
    g_devs[idx].select_slot = slot;
    if (idx + 1 > g_dev_count) g_dev_count = idx + 1;
    return true;
}

LAS10_Dev* LASM_FindByAddr(uint16_t mb_addr, uint16_t* offset)
{
    for (uint8_t i = 0; i < g_dev_count; ++i) {
        uint16_t base = g_devs[i].mb_base;
        if (mb_addr >= base && mb_addr < (uint16_t)(base + LAS_MB_BLOCK_SIZE)) {
            if (offset) *offset = (uint16_t)(mb_addr - base);
            return &g_devs[i];
        }
    }
    return NULL;
}

/* ===== 可选：发前清空RX，防粘包 ===== */
static void las_rx_flush(UART_HandleTypeDef* huart){
    uint8_t ch;
    while (HAL_UART_Receive(huart, &ch, 1, 0) == HAL_OK) { /* discard */ }
}

/* ===== 带 huart 参数的收帧（AA 55|LEN|...|CS） ===== */
static int recv_byte_on(UART_HandleTypeDef* huart, uint8_t* ch, uint32_t tout_ms) {
    return (HAL_UART_Receive(huart, ch, 1, tout_ms) == HAL_OK);
}
static bool las_recv_frame_on(UART_HandleTypeDef* huart, uint8_t* buf, uint16_t* out_len, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t b=0; int st=0;
    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (!recv_byte_on(huart, &b, 1)) continue;
        if (st==0){ if (b==0xAA){ buf[0]=b; st=1;} }
        else if (st==1){ if (b==0x55){ buf[1]=b; st=2; break;} else st=0; }
    }
    if (st!=2) return false;

    if (!recv_byte_on(huart, &buf[2], 3)) return false;   // LEN
    uint8_t len = buf[2];
    uint16_t need = (uint16_t)(len + 2);

    for (uint16_t i=0;i<need;++i)
        if (!recv_byte_on(huart, &buf[3+i], 3)) return false;

    uint8_t sum=0;
    for (uint16_t i=2;i<(uint16_t)(3+need-1);++i) sum += buf[i];
    if (sum != buf[3+need-1]) return false;

    *out_len = (uint16_t)(3 + need);
    return true;
}

/* ===== 通用一次事务：选通→发→收→释放 ===== */
static bool las_txrx(LAS10_Dev* dev, const uint8_t* tx, uint16_t tlen,
                     uint8_t* rx, uint16_t* rlen, uint32_t tout_ms)
{
    if (!dev || !dev->huart) return false;
    if (dev->select_fn) dev->select_fn(dev->select_slot, true);

    las_rx_flush(dev->huart); /* 可选：发前清Rx */
    if (HAL_UART_Transmit(dev->huart, (uint8_t*)tx, tlen, 10) != HAL_OK){
        if (dev->select_fn) dev->select_fn(dev->select_slot, false);
        return false;
    }
    bool ok = las_recv_frame_on(dev->huart, rx, rlen, tout_ms);
    if (dev->select_fn) dev->select_fn(dev->select_slot, false);
    return ok;
}


/* ===== 新增：字节接收与带帧头的收帧 ===== */

static int recv_byte(uint8_t* ch, uint32_t tout_ms) {
    return (HAL_UART_Receive(&huart1, ch, 1, tout_ms) == HAL_OK);
}

/* 等待应答：AA 55 | LEN | ... | CS
   out_len 返回 buf 中的有效长度（含 AA 55） */
static bool las_recv_frame(uint8_t* buf, uint16_t* out_len, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t b = 0;
    int state = 0;  // 0:找AA  1:找55  2:读LEN

    /* 1) 找帧头 AA 55 */
    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (!recv_byte(&b, 1)) continue;
        if (state == 0) { if (b == 0xAA) { buf[0] = b; state = 1; } }
        else if (state == 1) { if (b == 0x55) { buf[1] = b; state = 2; break; } else state = 0; }
    }
    if (state != 2) return false;

    /* 2) 取长度 (len = 数据字节数N + 2〔CMD+Index〕) */
    if (!recv_byte(&buf[2], 3)) return false;
    uint8_t len = buf[2];

    /* 3) 读取剩余 len+2 个字节（ID、CMD/Index/Data...、CS） */
    uint16_t need = (uint16_t)(len + 2);
    for (uint16_t i = 0; i < need; ++i) {
        if (!recv_byte(&buf[3 + i], 3)) return false;
    }

    /* 4) 校验：除帧头外所有字节的累加和低8位 == 最后一字节 */
    uint8_t sum = 0;
    for (uint16_t i = 2; i < (uint16_t)(3 + need - 1); ++i) sum += buf[i];
    if (sum != buf[3 + need - 1]) return false;

    *out_len = (uint16_t)(3 + need);
    return true;
}


/* 来自 CubeMX 的 huart1（电缸），以及微秒延时/超时使用的滴答 */
extern UART_HandleTypeDef huart1;

/* 当前电缸ID（默认1，可通过写寄存器改） */
static uint8_t s_las_id = 1;
/* 本地缓存，回显 Modbus 寄存器值的来源 */
static uint16_t s_goal_echo = 0;

/* ---------------- 实用函数：LAS 帧构造/校验/IO ---------------- */

static uint8_t las_checksum(const uint8_t* p, uint16_t len_no_hdr2B /*不含55 AA*/)
{
    uint32_t s = 0;
    for (uint16_t i = 0; i < len_no_hdr2B; ++i) s += p[i];
    return (uint8_t)(s & 0xFF);
}

/* 按手册：除帧头两字节外其余字节累加和低8位为校验和；应答头为 AA 55。:contentReference[oaicite:3]{index=3} */

static bool uart1_send(const uint8_t* d, uint16_t n)
{
    return (HAL_UART_Transmit(&huart1, (uint8_t*)d, n, 10) == HAL_OK);
}

/* 简单轮询接收：等待若干字节或到超时（微秒级近似用毫秒乘法足够） */
static int uart1_recv(uint8_t* d, uint16_t maxlen, uint32_t timeout_us)
{
    uint32_t t0 = HAL_GetTick();
    uint16_t got = 0;
    while ((HAL_GetTick() - t0) * 1000U < timeout_us)
    {
        uint8_t ch;
        if (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK)
        {
            if (got < maxlen) d[got++] = ch;
        }
    }
    return (int)got;
}

/* 解析“查询状态信息”应答帧（AA 55 ...），字段顺序按手册示例 */
static bool las_parse_status(const uint8_t* rx, uint16_t n, LAS10_Status* st)
{
    if (n < 22) return false;
    if (!(rx[0] == LAS_RSP_HDR1 && rx[1] == LAS_RSP_HDR2)) return false;
    /* 数据体长度 B2，ID B3，指令号 B4(=0x04)，参数1 B5(=0x00)，参数2 B6(=0x22) */
    /* 从 B7 起：按手册示例：
       B7-B8  目标位置(u16)
       B9-B10 当前位置(s16)
       B11    温度(int8)   —— 我们扩展存为 int16
       B12-B13 电流(u16 mA)
       B14    力低字节 (s16)
       B15    错误信息(1B)
       B16    力高字节
       B17-B18 内部1 (无意义)
       B19-B20 内部2 (无意义)
       B21    校验和
    */
    st->goal_pos   = (uint16_t)(rx[8]  << 8 | rx[7]);
    st->cur_pos    = (int16_t)(rx[10] << 8 | rx[9]);
    st->temp_c     = (int8_t)rx[11];
    st->current_mA = (uint16_t)(rx[13] << 8 | rx[12]);
    st->force_g    = (int16_t)((rx[16] << 8) | rx[14]); /* 注意中间夹了错误字节 */
    st->err_bits   = rx[15];
    return true;
}

/* 发送“单控”MC 指令，param2 取 LAS_MC_*，可选解析状态 */
static bool las_send_mc(uint8_t param2, LAS10_Status* out_st, uint32_t timeout_us)
{
    uint8_t tx[16];
    uint8_t nohdr[16];
    /* 格式参考手册示例：55 AA | LEN(=0x03) | ID | 0x04 | 0x00 | param2 | CS */
    uint16_t i = 0;
    nohdr[i++] = 0x03;       /* 长度 */
    nohdr[i++] = s_las_id;   /* ID */
    nohdr[i++] = LAS_CMD_MC; /* 指令号 */
    nohdr[i++] = 0x00;       /* 参数1（保留=0） */
    nohdr[i++] = param2;     /* 参数2 */

    uint8_t cs = las_checksum(nohdr, i);

    uint16_t t = 0;
    tx[t++] = LAS_HDR1; tx[t++] = LAS_HDR2;
    memcpy(&tx[t], nohdr, i); t += i;
    tx[t++] = cs;

    if (!uart1_send(tx, t)) return false;

//    if (out_st) {
//        /* 带状态返回的 MC(0x22 查询)、或者工作/急停也会回状态（按手册“参考查询返回”） */
//        uint8_t rx[64]; memset(rx, 0, sizeof(rx));
//        int got = uart1_recv(rx, sizeof(rx), timeout_us);
//        if (got < 10) return false;
//        return las_parse_status(rx, (uint16_t)got, out_st);
//    }
if (out_st) {
    uint8_t rx[64]; uint16_t n = 0;
    if (!las_recv_frame(rx, &n, 20)) return false;  // 20ms 窗口
    return las_parse_status(rx, n, out_st);
}

    return true; /* 无需状态 */
}

/* 发送定位模式（带反馈）写目标位置 */
static bool las_move_fb(uint16_t pos, LAS10_Status* out_st, uint32_t timeout_us)
{
    uint8_t tx[16];
    uint8_t nohdr[16];
    /* 示例：55 AA 04 ID 0x21 0x37  posL posH CS */
    uint16_t i = 0;
    nohdr[i++] = 0x04;         /* 长度 */
    nohdr[i++] = s_las_id;     /* ID */
    nohdr[i++] = LAS_CMD_MOVE_FB; /* 0x21 */
    nohdr[i++] = LAS_IDX_GOAL_POS; /* 0x37 */
    nohdr[i++] = (uint8_t)(pos & 0xFF);      /* 低字节 */
    nohdr[i++] = (uint8_t)((pos >> 8) & 0xFF);/* 高字节 */

    uint8_t cs = las_checksum(nohdr, i);

    uint16_t t = 0;
    tx[t++] = LAS_HDR1; tx[t++] = LAS_HDR2;
    memcpy(&tx[t], nohdr, i); t += i;
    tx[t++] = cs;

    if (!uart1_send(tx, t)) return false;

//    if (out_st) {
//        uint8_t rx[64]; memset(rx, 0, sizeof(rx));
//        int got = uart1_recv(rx, sizeof(rx), timeout_us);
//        if (got < 10) return false;
//        return las_parse_status(rx, (uint16_t)got, out_st);
//    }
if (out_st) {
    uint8_t rx[64]; uint16_t n = 0;
    if (!las_recv_frame(rx, &n, 20)) return false;
    return las_parse_status(rx, n, out_st);
}

    return true;
}

/* 写控制表（WR）：索引+2字节值通用写入 */
static bool las_wr_u16(uint8_t index, uint16_t value)
{
    /* 55 AA LEN ID 0x02 index valL valH CS */
    uint8_t tx[16], nohdr[16];
    uint16_t i = 0;
    nohdr[i++] = 0x04;
    nohdr[i++] = s_las_id;
    nohdr[i++] = LAS_CMD_WR;
    nohdr[i++] = index;
    nohdr[i++] = (uint8_t)(value & 0xFF);
    nohdr[i++] = (uint8_t)((value >> 8) & 0xFF);
    uint8_t cs = las_checksum(nohdr, i);
    uint16_t t = 0;
    tx[t++] = LAS_HDR1; tx[t++] = LAS_HDR2;
    memcpy(&tx[t], nohdr, i); t += i;
    tx[t++] = cs;
    return uart1_send(tx, t);
}

/* ---------------- 对外 API ---------------- */

void LAS10_Init(uint8_t las_id)
{
    s_las_id = las_id;
    s_goal_echo = 0;
}

bool LAS10_SendMC(uint8_t param2, LAS10_Status* out_status, uint32_t timeout_us)
{
    return las_send_mc(param2, out_status, timeout_us);
}

bool LAS10_SetGoalPos(uint16_t pos, LAS10_Status* out_status, uint32_t timeout_us)
{
    s_goal_echo = pos;
    return las_move_fb(pos, out_status, timeout_us);
}

bool LAS10_QueryStatus(LAS10_Status* out_status, uint32_t timeout_us)
{
    return las_send_mc(LAS_MC_QUERY, out_status, timeout_us);
}

/* 参数写：索引参考手册控制表（ID=偏移2，波特率=12，过流=32~33，过温=98~99，回温=100~101）。:contentReference[oaicite:4]{index=4} */
bool LAS10_WriteID(uint8_t new_id)      { return las_wr_u16(2, new_id); }
bool LAS10_WriteBaudCode(uint8_t code)  { return las_wr_u16(12, code);  }
bool LAS10_WriteOCP(uint16_t mA)        { return las_wr_u16(32, mA);    }
bool LAS10_WriteOTP_x10C(uint16_t x10C) { return las_wr_u16(98, x10C);  }
bool LAS10_WriteResume_x10C(uint16_t x10C) { return las_wr_u16(100, x10C); }

/* ---------------- 和你现有 Modbus 层打通 ---------------- */

extern uint16_t MODBUS_Reg[];   /* 你的寄存器数组，来自 modbus.c */

static void las_status_to_regs(const LAS10_Status* st)
{
    MODBUS_Reg[MB_ADDR_LAS_GOAL_POS]   = st->goal_pos;
    MODBUS_Reg[MB_ADDR_LAS_GOAL_ECHO]  = s_goal_echo;
    MODBUS_Reg[MB_ADDR_LAS_CUR_POS]    = (uint16_t)st->cur_pos;
    MODBUS_Reg[MB_ADDR_LAS_TEMP_C]     = (uint16_t)st->temp_c;
    MODBUS_Reg[MB_ADDR_LAS_CURRENT_MA] = st->current_mA;
    MODBUS_Reg[MB_ADDR_LAS_FORCE_G]    = (uint16_t)st->force_g;
    MODBUS_Reg[MB_ADDR_LAS_ERR_BITS]   = st->err_bits;
}



/* 把状态写回本地MODBUS_Reg（base为块基址）*/
static void las_status_to_regs_at(uint16_t base, const LAS10_Status* st, uint16_t goal_echo)
{
    MODBUS_Reg[base + MB_ADDR_LAS_GOAL_POS]   = st->goal_pos;
    MODBUS_Reg[base + MB_ADDR_LAS_GOAL_ECHO]  = goal_echo;
    MODBUS_Reg[base + MB_ADDR_LAS_CUR_POS]    = (uint16_t)st->cur_pos;
    MODBUS_Reg[base + MB_ADDR_LAS_TEMP_C]     = (uint16_t)st->temp_c;
    MODBUS_Reg[base + MB_ADDR_LAS_CURRENT_MA] = st->current_mA;
    MODBUS_Reg[base + MB_ADDR_LAS_FORCE_G]    = (uint16_t)st->force_g;
    MODBUS_Reg[base + MB_ADDR_LAS_ERR_BITS]   = st->err_bits;
		// las10_bridge.c 的 las_status_to_regs_at() 末尾加：
	uint8_t idx = (uint8_t)(base / LAS_MB_BLOCK_SIZE); // base=0x0000/0x0100/...
	if (idx < 4) {
	int16_t cp = st->cur_pos;
	if (cp < 0) cp = 0;
	MODBUS_Reg[MB_FB_LAS_POS1 + idx] = (uint16_t)cp;
	}
	int16_t cp = st->cur_pos;
if (cp < 0) cp = 0;

int16_t goal = (int16_t)goal_echo;
if (goal >= 0) {
    int16_t d = cp - goal;
    if (d < 0) d = -d;
    if (d <= 2) cp = goal;   // 吸附到目标
	
}
MODBUS_Reg[MB_FB_LAS_POS1 + idx] = (uint16_t)cp;

}
void LASM_OnWriteSingle(uint16_t mb_addr, uint16_t value)
{
    static uint16_t g_grp_mask = 0x000F; // 默认全选 #1~#4

    /* --- ① 先处理 Group 全局寄存器 --- */
    if (mb_addr == MB_GRP_MASK) {
        g_grp_mask = value;
        MODBUS_Reg[mb_addr] = value;
        return;
    }
    if (mb_addr == MB_GRP_CMD) {
        for (uint8_t i = 0; i < g_dev_count; ++i) {
            if ((g_grp_mask & (1u << i)) && g_devs[i].huart) {
                LAS10_Status st;
                if (LAS10_SendMC_Dev(&g_devs[i], (uint8_t)(value & 0xFF), &st, 3000)) {
                    las_status_to_regs_at(g_devs[i].mb_base, &st,
                        MODBUS_Reg[g_devs[i].mb_base + MB_ADDR_LAS_GOAL_POS]);
                }
                HAL_Delay(1); // 最小1ms间隔，避免总线拥挤
            }
        }
        MODBUS_Reg[mb_addr] = value;
        return;
    }
    if (mb_addr == MB_GRP_GOAL) {
        for (uint8_t i = 0; i < g_dev_count; ++i) {
            if ((g_grp_mask & (1u << i)) && g_devs[i].huart) {
                LAS10_Status st;
                if (LAS10_SetGoalPos_Dev(&g_devs[i], value, &st, 20000)) {
                    las_status_to_regs_at(g_devs[i].mb_base, &st, value);
                }
                HAL_Delay(1);
            }
        }
        MODBUS_Reg[mb_addr] = value;
        return;
    }
	/* ---- 组播多目标：写四个影子寄存器 ---- */
if (mb_addr >= MB_GRP_MPOS1 && mb_addr <= MB_GRP_MPOS4) {
    uint8_t idx = (uint8_t)(mb_addr - MB_GRP_MPOS1);  // 0..3 -> #1..#4
    g_grp_pos[idx] = value;
    MODBUS_Reg[mb_addr] = value;  // 本地回显
    return;
}
/* ---- 提交：把影子值一次性分发到被选择的电缸 ---- */
if (mb_addr == MB_GRP_APPLY && value == 1) {
    extern uint16_t MODBUS_Reg[];          // 你已有
    uint16_t mask = MODBUS_Reg[MB_GRP_MASK];  // bit0..3 选择 #1..#4
    for (uint8_t i = 0; i < g_dev_count; ++i) {
        if ((mask & (1u << i)) && g_devs[i].huart) {
            LAS10_Status st;
            uint16_t pos = g_grp_pos[i];
            if (LAS10_SetGoalPos_Dev(&g_devs[i], pos, &st, 20000)) {
                /* 刷回对应块的寄存器（与你现有的函数一致） */
                las_status_to_regs_at(g_devs[i].mb_base, &st, pos);
            }
            HAL_Delay(1); // 1ms 节流，避免总线拥挤
        }
    }
    MODBUS_Reg[mb_addr] = 1;  // 回显提交
    return;
}



    /* --- ② 常规：单设备写入（你的原逻辑，保留） --- */
    uint16_t off=0; LAS10_Dev* dev = LASM_FindByAddr(mb_addr, &off);
    if (!dev) return;

    LAS10_Status st; bool ok=false;
    switch (off)
    {
    case MB_ADDR_LAS_CMD:
        ok = LAS10_SendMC_Dev(dev, (uint8_t)(value & 0xFF), &st, 3000);
        if (ok) las_status_to_regs_at(dev->mb_base, &st,
            MODBUS_Reg[dev->mb_base + MB_ADDR_LAS_GOAL_POS]);
        break;

    case MB_ADDR_LAS_GOAL_POS:
        MODBUS_Reg[mb_addr] = value; /* 回显 */
        ok = LAS10_SetGoalPos_Dev(dev, value, &st, 3000);
        if (ok) las_status_to_regs_at(dev->mb_base, &st, value);
        break;

    default:
        MODBUS_Reg[mb_addr] = value;
        break;
    }
}

void LASM_OnWriteBlock(uint16_t mb_addr_start, const uint16_t* data, uint16_t len)
{
    for (uint16_t k=0;k<len;++k)
        LASM_OnWriteSingle(mb_addr_start + k, data[k]);
}

void LASM_OnReadMaybeRefresh(uint16_t mb_addr_start, uint16_t qty)
{
    uint16_t a0 = mb_addr_start, a1 = (uint16_t)(mb_addr_start + qty - 1);
    for (uint8_t i=0;i<g_dev_count;++i) {
        uint16_t base = g_devs[i].mb_base;
        uint16_t b0 = base, b1 = (uint16_t)(base + LAS_MB_BLOCK_SIZE - 1);
        if (!(a1 < b0 || a0 > b1)) {
            LAS10_Status st;
            if (LAS10_QueryStatus_Dev(&g_devs[i], &st, 3000)) {
                las_status_to_regs_at(base, &st, MODBUS_Reg[base + MB_ADDR_LAS_GOAL_POS]);
            }
        }
    }
}


bool LAS10_SendMC_Dev(LAS10_Dev* dev, uint8_t param2, LAS10_Status* out, uint32_t timeout_us)
{
    if (!dev || !dev->huart) return false;
    uint8_t nohdr[16], tx[16]; uint16_t i=0, t=0;

    nohdr[i++] = 0x03;
    nohdr[i++] = dev->id;
    nohdr[i++] = LAS_CMD_MC;
    nohdr[i++] = 0x00;
    nohdr[i++] = param2;
    uint8_t cs = las_checksum(nohdr, i);

    tx[t++]=LAS_HDR1; tx[t++]=LAS_HDR2; memcpy(&tx[t], nohdr, i); t+=i; tx[t++]=cs;

    if (!out) {
        return (HAL_UART_Transmit(dev->huart, tx, t, 10) == HAL_OK);
    } else {
        uint8_t rx[64]; uint16_t n=0;
        if (!las_txrx(dev, tx, t, rx, &n, (timeout_us+999)/1000)) return false;
        return las_parse_status(rx, n, out);
    }
}

bool LAS10_SetGoalPos_Dev(LAS10_Dev* dev, uint16_t pos, LAS10_Status* out, uint32_t timeout_us)
{
    if (!dev || !dev->huart) return false;
    uint8_t nohdr[16], tx[16]; uint16_t i=0, t=0;

    nohdr[i++] = 0x04;
    nohdr[i++] = dev->id;
    nohdr[i++] = LAS_CMD_MOVE_FB;
    nohdr[i++] = LAS_IDX_GOAL_POS;
    nohdr[i++] = (uint8_t)(pos & 0xFF);
    nohdr[i++] = (uint8_t)((pos >> 8) & 0xFF);
    uint8_t cs = las_checksum(nohdr, i);

    tx[t++]=LAS_HDR1; tx[t++]=LAS_HDR2; memcpy(&tx[t], nohdr, i); t+=i; tx[t++]=cs;

    if (!out) {
        return (HAL_UART_Transmit(dev->huart, tx, t, 10) == HAL_OK);
    } else {
        uint8_t rx[64]; uint16_t n=0;
        if (!las_txrx(dev, tx, t, rx, &n, (timeout_us+999)/1000)) return false;
        return las_parse_status(rx, n, out);
    }
}

bool LAS10_QueryStatus_Dev(LAS10_Dev* dev, LAS10_Status* out, uint32_t timeout_us)
{
    return LAS10_SendMC_Dev(dev, LAS_MC_QUERY, out, timeout_us);
}

void LASM_Poll_50ms(void)
{
    static uint32_t tick = 0;
    static uint8_t rr = 0;
    if (HAL_GetTick() - tick < 20) return;
    tick = HAL_GetTick();

    if (g_dev_count == 0) return;
    if (rr >= g_dev_count) rr = 0;

    LAS10_Status st;
    if (LAS10_QueryStatus_Dev(&g_devs[rr], &st, 100000)) {
        // 这里会连同 Telemetry(0x0420..) 一起更新
        las_status_to_regs_at(g_devs[rr].mb_base, &st,MODBUS_Reg[g_devs[rr].mb_base + MB_ADDR_LAS_GOAL_POS]);
    }
    rr++;
}


