/*
 * sfra_cfg.h
 *
 * SFRA 扫频（TI SFRA_IQ v1.20）的配置与对外声明。
 *
 * SFRA_ENABLE == 1 时才引入 SFRA 库；== 0 时只提供"什么都不做"的注入/采集宏，
 * 这样 LOOP_MODE=1/2/3/4 这些分支在 OLED 版里也能正常编译运行。
 *
 * 点数怎么定（2026-10-29 实测定标，别随手加大）：
 *   采样/中断是 20kHz，SFRA 的有效上限大致到 fsw/10 = 2kHz。
 *   每个频点的耗时是 SFRA_IQ 库内部写死的：
 *       preCount 32768 拍（预置：等被控对象在新频率上稳定）
 *     + 测量窗口至少 32768 拍（count=0x7FFF，之后还要等相位回到起点）
 *     => 65536 次中断 / 20kHz = 3.28 s/点（反汇编已核实两处都是 0x7FFF）
 *   现在不接上位机（SFRA_GUI_ENABLE=0，扫频由 CCS 的 SfraTrig 触发），
 *   所以没有"SFRA GUI 约 60 s 就判 Connection Lost"这个限制，点数按需要取：
 *       N=48 -> 100Hz ~ 2.0kHz（约 36 点/十倍频程），总时长约 157 s
 *   ★ 改点数必须同时改 SFRA_FREQ_STEP = (fmax/fmin)^(1/(N-1))，
 *     否则频率轴和扫频上限会整体偏掉。
 *   常用几组（fmin=100Hz, fmax=2kHz，括号里是 20kHz 中断下的总时长）：
 *       N=14  1.259155  (46 s)      N=24  1.139112  (79 s)
 *       N=32  1.101460  (105 s)     N=48  1.065814  (157 s)
 */

#ifndef SFRA_CFG_H
#define SFRA_CFG_H

#include "dcac_cfg.h"

#if (SFRA_ENABLE == 1)

#include "C:/ti/controlSUITE/libs/app_libs/SFRA/v1_20_00_00/IQ/include/SFRA_IQ_Include.h"

/* ---- 扫频参数 ----
 * 20kHz 中断下 3.28 s/点，48 点约 157 s（不接 GUI，没有超时顾虑）。
 * 扫完 SfraDone 变 1，5 个数组里就是完整结果，直接导走即可。
 * 点数/步进的时间表见文件头。
 */
#define SFRA_FREQ_LENGTH    100         /* 点数: 100Hz ~ 5.0kHz, 约 328 s */
#define SFRA_FREQ_START     100.0f      /* 起始频率 Hz */
#define SFRA_FREQ_STEP      1.040307f   /* = 50^(1/99)：100 点正好 100Hz->5.0kHz */
#define SFRA_ISR_FREQ       20000.0f    /* 中断频率 = 开关频率，填错频率轴整体偏移 */

/* ---- 自动开始扫频（不用在 CCS 里手写 SfraTrig） ----
 * ★ 为什么必须这么做：SfraTrig 是 .ebss 里的普通变量，_c_int00 启动时会把它清零，
 *   所以在 Expressions 里"先改成 1 再下载"是没用的 —— 一跑就被清回 0。
 *   这里改成编译期常量：SFRA_AUTO_START=1 时，程序跑到主循环、等软启动爬完后
 *   （SFRA_AUTO_START_DELAY_MS）自动触发一次扫频，下载完就自己开扫。
 *   想再扫一遍：程序运行起来之后在 Expressions 里写 SfraTrig=1（这次不会被清）。
 */
#ifndef SFRA_AUTO_START
#define SFRA_AUTO_START            1
#endif
#define SFRA_AUTO_START_DELAY_MS   2000U   /* 软启动只有 500ms，留 4 倍余量 */

/* 延时折算成中断次数：中断频率 = TBCLK/(2*TBPRD) = 20kHz -> 1ms = 20 次中断 */
#define SFRA_AUTO_START_ISRCNT     \
    ((Uint16)((SFRA_AUTO_START_DELAY_MS * (TBCLK_Hz / (2UL * TBPRD))) / 1000UL))

/* ---- 上位机 SCI ---- */
#define SFRA_SCI_LSPCLK     15000000L   /* LSPCLK = SYSCLK/2 = 15MHz */
#define SFRA_SCI_BAUDRATE   57600L      /* SFRA GUI 默认 */

/* ---- 扫频工作点（参考冻结成直流，不走 50Hz 正弦）----
 * ★ SFRA_ENABLE=1 时四种环路模式全部用直流工作点，见 dcac_cfg.h 的"环路模式"
 *   SFRA_DUTY_DC  给模式 0（注入占空比，采 Mod->IL = 受控体本身）
 *   SFRA_IREF_DC  给模式 2（电流单环）
 *   SFRA_VREF_DC  给模式 1/3（含电压环）
 */
#define SFRA_VREF_DC        _IQ(20.0f)  /* 模式 1/3：输出电压直流给定 20V */
#define SFRA_DUTY_DC        _IQ(0.40f)  /* 模式 0：固定占空比 0.4（量受控体） */
#define SFRA_IREF_DC        _IQ(1.0f)   /* 模式 2：直流电流给定 1A */

/* ---- 库对象与结果数组 ---- */
extern SFRA_IQ  SFRA1;
extern float32  FreqVect[SFRA_FREQ_LENGTH];
extern int32    Plant_MagVect[SFRA_FREQ_LENGTH];
extern int32    Plant_PhaseVect[SFRA_FREQ_LENGTH];
extern int32    OL_MagVect[SFRA_FREQ_LENGTH];
extern int32    OL_PhaseVect[SFRA_FREQ_LENGTH];

#if (SFRA_GUI_ENABLE == 1)
/* ---- 上位机映射表（顺序由 SFRA GUI 写死，不可改） ---- */
extern int16   *varSetTxtList[16];
extern int16   *varSetBtnList[16];
extern int16   *varSetSldrList[16];
extern int16   *varGetList[16];
extern int32   *arrayGetList[16];
extern int32   *dataGetList[16];
extern Uint32  *dataSetList[16];

extern int16    CommsOKflg;
extern int16    SerialCommsTimer;
extern int16    initializationFlag;
#endif  /* SFRA_GUI_ENABLE */

/* ---- 函数 ---- */
extern void SFRA_Init(void);
#if (SFRA_GUI_ENABLE == 1)
extern void SCIA_Init(long SCI_VBUS_CLOCKRATE, long SCI_BAUDRATE);
extern void SerialHostComms(void);
#endif

/* ---- 不用 GUI 时的扫频触发/完成标志（CCS Expressions 里写/看） ---- */
extern volatile int16 SfraTrig;   /* 写 1 -> 启动一次扫频 */
extern volatile int16 SfraDone;   /* 1 = 扫完了, 5 个数组里是完整结果 */

/* ---- 注入/采集 ----
 * 和 Boost 保持一致：ISR 里无条件调用 SFRA_IQ_INJECT / SFRA_IQ_COLLECT。
 * Boost 里没有故障门 —— 库内部按 start 自管相位，跳过 INJECT 会让相位停住
 * 而 COLLECT 还在累加，扫出来的数据是错的。保护由 Protect 模块单独负责。
 */
#define SFRA_INJ(dc)        ((int32)SFRA_IQ_INJECT((int32)(dc)))

#define SFRA_COL(in, fdb)                                                       \
    SFRA_IQ_COLLECT((volatile int32 *)(in), (volatile int32 *)(fdb))

#else   /* SFRA_ENABLE == 0 */

/* 不扫频时：注入就是直接给直流工作点，采集什么都不做。
   为什么必须有：LOOP_MODE=1/2/3/4 的分支在不扫频时也会编译到 SFRA_INJ，
   宏不存在的话 C 会当成隐式函数调用，链接时报 _SFRA_INJ 未解析。 */
#define SFRA_INJ(dc)        ((int32)(dc))
#define SFRA_COL(in, fdb)   ((void)0)

#endif   /* SFRA_ENABLE */

#endif   /* SFRA_CFG_H */
