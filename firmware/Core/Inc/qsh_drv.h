#ifndef __QSH_DRV_H__
#define __QSH_DRV_H__

#include "main.h"
#include "qsh_frame.h"

/* QSH-H 数字称重传感器双通道驱动。
   通道 A（参考，砝码槽）= USART3；通道 B（称量，称盘）= USART2。

   与交接版 v1 的差异（对应《算法设计-A轨》）：
   1. 读**内码**（协议地址 0x17，32 位原码）而非 40001 净重值——绕过模块
      侧零点管理（开机置零/清零/零点跟踪），host 全权管理零点语义；
   2. 每帧盖 µs 时间戳（对齐算法的时间基准）；
   3. 上电对两模块下发会话配置：40005=1（开机不置零）、40006=0（滤波弱）、
      40007=0（蠕动关），带回显确认与重试；
   4. 不再实现模块侧"清零"指令（按键语义移交 host 层）。 */

#define QSH_CH_A_REF   0U    /* 参考通道：USART3 */
#define QSH_CH_B_MEAS  1U    /* 称量通道：USART2 */
#define QSH_NCH        2U

typedef struct
{
  uint8_t  ch;        /* QSH_CH_A_REF / QSH_CH_B_MEAS */
  int32_t  code;      /* 内码（32 位原码） */
  uint64_t tUs;       /* 帧完成时刻（µs，公共时间轴） */
  uint8_t  dup;       /* 与上一响应同值（同一次转换的重复轮询） */
  uint32_t seq;       /* 通道内响应序号（诊断用） */
} QshSample_t;

typedef struct
{
  uint32_t readOk;
  uint32_t crcErr;
  uint32_t frameErr;
  uint32_t txBusy;
  uint32_t cfgAck;
  uint32_t cfgFail;
  uint32_t zeroAck;      /* 清零指令回显计数 */
  uint32_t ringDrop;
} QshStat_t;

/* —— 调试观测用全局变量（qsh_drv.c 定义，非 static，调试器可直接 Watch）—— */
/* 正常模式：算法常驻，环=128（3KB 通信抖动余量）。
   录制模式（运行时切换，无需重编译）：调试器写 g_recArm=1 →
   main 暂停算法并调 QshDrv_RecBegin(&g_scale, sizeof(g_scale))
   （~57KB → 2389 条 ≈ 2 分钟@20样本/s）。调试器 SAVE 从 g_recBufAddr
   导出 g_recCount×24B → qsh_bin2csv.py 转 CSV。g_recArm=0 → 算法重启。 */
#define QSH_RING_N 128U

extern uint8_t       s_cmdRead[QSH_NCH][8];    /* 读内码命令 */
extern uint8_t       s_cmdCfg[QSH_NCH][3][8];  /* 配置写命令 */
extern QshFrame_t    s_frame[QSH_NCH];         /* 帧层（s_frame[1].frame[] = 串口2原始字节） */
extern QshSample_t * volatile s_ring;                   /* 当前环形缓冲指针（正常=小环，录制=算法内存） */
extern volatile uint16_t s_ringCap;                /* 当前容量（条数） */
extern uint16_t      s_head, s_tail;           /* 环形缓冲下标 */
extern int32_t       s_lastCode[QSH_NCH];      /* ★ 各通道最新内码 [0]=A参考(串口3) [1]=B测量(串口2) */
extern uint8_t       s_hasLast[QSH_NCH];       /* 已收到首帧 */
extern uint32_t      s_seq[QSH_NCH];           /* 通道内响应序号 */
extern uint32_t      s_lastTick[QSH_NCH];      /* 最新值时刻(HAL tick) */
extern volatile uint8_t s_ready;               /* 配置完成，轮询进行中 */
extern volatile QshStat_t s_stat[QSH_NCH];     /* 各通道计数器 */

/* —— RAM 录制（运行时切换：写 g_recArm=1 开始 → SAVE 导出 → 写 0 算法重启）—— */
extern volatile uint8_t g_recArm;              /* 调试器写 1=开始录制，0=停止+算法重启 */
extern volatile uint8_t g_recFull;             /* 录满置 1 并冻结（调试器读）*/
extern volatile uint32_t g_recBufAddr;         /* 录制缓冲起始地址（SAVE 参数）*/
extern volatile uint32_t g_recCount;           /* 已录样本数（SAVE 长度=g_recCount×24B）*/
extern uint8_t  g_recAlgoHold;                 /* 1=算法暂停（录制中），main 监控 */

void QshDrv_RecBegin(void *buf, uint32_t bytes);  /* main 调（g_recArm↑沿）*/
void QshDrv_RecEnd(void);                         /* main 调（g_recArm↓沿/录满）*/

/* —— 调试器寄存器探针（单次 MODBUS 读/写，Watch 驱动；标定/诊断用）——
   读：设 g_probeCh/g_probeReg/g_probeCnt(1..10)，写 g_probeOp=1
       → 成功后 g_probeDone=0xA5，数据在 g_probeData[0..cnt-1]（每寄存器 uint16 大端）
   写06：设 g_probeCh/g_probeReg/g_probeVal，写 g_probeOp=2 → 0xA5=模块已回显
   写16(32位)：g_probeVal=高字、g_probeData[0]=低字，写 g_probeOp=3
   0xAE=模块异常(见g_probeExc) 0x99=超时 0x98=参数错；写超时 1.5s；
   超时后若 g_probeLate 变 1 = 回显迟到、写实际已生效 */
extern volatile uint8_t  g_probeOp;            /* 0=空闲 1=读 2=写06 3=写16双寄存器（固件完成即清 0） */
extern volatile uint8_t  g_probeCh;            /* 0=A参考 1=B测量 */
extern volatile uint16_t g_probeReg;           /* 协议地址（如 0x0D=满量程高字） */
extern volatile uint16_t g_probeCnt;           /* 读寄存器个数 1..10 */
extern volatile uint16_t g_probeVal;           /* 写入值 / op3 时为 32 位高字 */
extern volatile uint16_t g_probeData[10];      /* 读结果 / op3 时 [0]=32 位低字 */
extern volatile uint8_t  g_probeDone;          /* 0=进行中 0xA5=成功 0xAE=模块异常 0x99=超时 0x98=参数错 */
extern volatile uint8_t  g_probeRx[16];        /* 探针期间收到的原始帧（诊断） */
extern volatile uint8_t  g_probeRxLen;         /* 原始帧长度 */
extern volatile uint8_t  g_probeExc;           /* MODBUS 异常码: 01非法功能 02非法地址 03非法数值 04内部故障 */
extern volatile uint8_t  g_probeLate;          /* 1=超时后收到迟到回显 */

void QshDrv_Init(void);                       /* 构建命令、启动收帧与配置状态机 */
void QshDrv_Process(void);                    /* 主循环调用：取帧/解析/入环/启动配置 */
uint8_t QshDrv_Ready(void);                   /* 1 = 配置完成，轮询进行中 */
void QshDrv_SendZero(uint8_t ch);             /* 发模块清零指令（M1 内码独立性验证用） */
int QshDrv_PopSample(QshSample_t *out);       /* 取一条样本（主循环消费），无则 0 */
int QshDrv_Last(uint8_t ch, int32_t *code, uint32_t *ageMs);  /* 查询最新内码与数据年龄 */
void QshDrv_GetStats(QshStat_t out[QSH_NCH]);
void QshDrv_IdleIRQ(UART_HandleTypeDef *huart);   /* stm32f4xx_it.c 的 USART IRQ 调用 */

#endif
