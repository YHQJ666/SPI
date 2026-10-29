/*
 * sfra_task.c
 *
 * SFRA 扫频（TI SFRA_IQ v1.20）的初始化、库对象、结果数组和后台状态机。
 * 从 main.c 搬出来的：扫频是调试功能，和 20kHz 控制环无关，
 * 平时 SFRA_ENABLE=0 时这个文件整个编成空壳。
 *
 * 引脚：GPIO28 = SCIRXDA，GPIO29 = SCITXDA（和 OLED 是同一对脚，只能二选一）
 * 配置宏在 sfra_cfg.h，注入/采集宏在中断里用。
 */

#include "oled_task.h"
#include "sfra_task.h"
#include "dcac_cfg.h"
#include "sfra_cfg.h"

#if (SFRA_ENABLE == 1)

// 主循环里的中断计数（main.c 里定义），自动扫频延时判据要用
extern volatile Uint16 IsrCnt;
//===========================================================================
// SFRA 扫频：库对象、结果数组、上位机映射表、SCI-A
//   引脚：GPIO28 = SCIRXDA，GPIO29 = SCITXDA（和 OLED 是同一对脚，只能二选一）
//   波特率 57600 @ LSPCLK 15MHz
//===========================================================================
SFRA_IQ  SFRA1;

float32  FreqVect[SFRA_FREQ_LENGTH];
int32    Plant_MagVect[SFRA_FREQ_LENGTH];
int32    Plant_PhaseVect[SFRA_FREQ_LENGTH];
int32    OL_MagVect[SFRA_FREQ_LENGTH];
int32    OL_PhaseVect[SFRA_FREQ_LENGTH];

#if (SFRA_GUI_ENABLE == 1)
// 上位机映射表：下标顺序由 SFRA GUI 写死，不能改
int16   *varSetTxtList[16];
int16   *varSetBtnList[16];
int16   *varSetSldrList[16];
int16   *varGetList[16];
int32   *arrayGetList[16];
int32   *dataGetList[16];
Uint32  *dataSetList[16];

int16    CommsOKflg       = 0;
int16    SerialCommsTimer  = 0;   // 1ms 递增，串口超时用
int16    initializationFlag = 0;  // GUI 点 Start 时置 1

static Uint16 SfraTickCnt = 0U;

// 上位机协议没用到的三张表指向这里，防止空指针写
static int16  SfraDummyTxt[16];
static int16  SfraDummySldr[16];
static int32  SfraDummyGet[16];
#endif  /* SFRA_GUI_ENABLE */

/* ---- 扫频触发/完成标志 ----
 * 触发有两条路，都不需要"下载前置 1"（.ebss/.cinit 在启动时都会被初始化覆盖）：
 *   1) SFRA_AUTO_START=1（默认）：上电/下载后延时 SFRA_AUTO_START_DELAY_MS 自动扫一遍；
 *   2) 程序跑起来之后，在 Expressions 里把 SfraTrig 写 1（运行中写才有效）。
 * ★ 千万不要给 SfraTrig 写初值 1：那样上电瞬间就开扫，RefRamp 软启动还没爬完，
 *   第一个频点的数据是废的（这正是这里两个变量都写 0 的原因）。
 * 观察：SFRA1.FreqIndex 从 0 涨到 SFRA1.Vec_Length（48 点约 157 s）；
 *       SfraDone 变 1 = 扫完了，5 个数组里就是完整结果。
 * 一个频点固定 65536 个中断（库内部 preCount+窗口），20kHz 下 3.28 s。
 */
volatile int16 SfraTrig = 0;
volatile int16 SfraDone = 0;

#if (SFRA_AUTO_START == 1)
static Uint16 SfraAutoFired = 0U;   /* 自动启动只触发一次 */
#endif

#if (SFRA_GUI_ENABLE == 1)
static void SCIA_GPIO_Init(void)
{
    EALLOW;

    GpioCtrlRegs.GPAPUD.bit.GPIO28  = 0;    // RX 开上拉，悬空不乱收噪声
    GpioCtrlRegs.GPAPUD.bit.GPIO29  = 1;    // TX 输出，关上拉
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;    // SCIRXDA
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;    // SCITXDA
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;   // 异步输入（串口必须）

    EDIS;
}
#endif  /* SFRA_GUI_ENABLE */

void Sfra_Task_Init(void)
{
    // 注入幅值（Q26，按信号自己的单位）：
    //   固定占空比  -> 占空比 ±2%
    //   电流给定    -> ±0.3A
    //   电压给定    -> ±0.5V
#if (LOOP_MODE == LOOP_FIXED_DUTY)
    SFRA1.amplitude = _IQ26(0.02f);
#elif (LOOP_MODE == LOOP_CURR_CLOSED)
    SFRA1.amplitude = _IQ26(0.30f);
#else
    SFRA1.amplitude = _IQ26(0.50f);
#endif

    SFRA1.Vec_Length = SFRA_FREQ_LENGTH;
    SFRA1.ISR_Freq   = SFRA_ISR_FREQ;
    SFRA1.Freq_Start = SFRA_FREQ_START;
    SFRA1.Freq_Step  = SFRA_FREQ_STEP;

    SFRA1.FreqVect     = FreqVect;
    SFRA1.GH_MagVect   = OL_MagVect;
    SFRA1.GH_PhaseVect = OL_PhaseVect;
    SFRA1.H_MagVect    = Plant_MagVect;
    SFRA1.H_PhaseVect  = Plant_PhaseVect;

#if (SFRA_GUI_ENABLE == 1)
    initializationFlag = 0;
#endif
    SFRA_IQ_INIT(&SFRA1);

#if (SFRA_GUI_ENABLE == 1)
    // GUI <-> 变量映射
    varGetList[0] = (int16 *)&(SFRA1.Vec_Length);
    varGetList[1] = (int16 *)&(SFRA1.status);
    varGetList[2] = (int16 *)&(SFRA1.FreqIndex);

    dataSetList[0] = (Uint32 *)&(SFRA1.Freq_Start);   // float32
    dataSetList[1] = (Uint32 *)&(SFRA1.amplitude);    // int32 (Q26)
    dataSetList[2] = (Uint32 *)&(SFRA1.Freq_Step);    // float32

    varSetBtnList[0] = (int16 *)&initializationFlag;

    // 命令 1/3/6 会解引用 varSetTxtList / varSetSldrList / dataGetList，
    // 这三张表本工程没用到，但不能留空指针——静默的 BSS 是 0，
    // 串口噪声凑出一个命令 1 就会往地址 0 写。统一指向哨兵变量。
    {
        Uint16 k;
        for (k = 0U; k < 16U; k++)
        {
            varSetTxtList[k]  = &SfraDummyTxt[k];
            varSetSldrList[k] = &SfraDummySldr[k];
            dataGetList[k]    = &SfraDummyGet[k];
        }
    }

    arrayGetList[0] = (int32 *)FreqVect;
    arrayGetList[1] = (int32 *)OL_MagVect;
    arrayGetList[2] = (int32 *)OL_PhaseVect;
    arrayGetList[3] = (int32 *)Plant_MagVect;
    arrayGetList[4] = (int32 *)Plant_PhaseVect;
    arrayGetList[5] = (int32 *)&(SFRA1.Freq_Start);
    arrayGetList[6] = (int32 *)&(SFRA1.amplitude);
    arrayGetList[7] = (int32 *)&(SFRA1.Freq_Step);

    SCIA_GPIO_Init();
    SCIA_Init(SFRA_SCI_LSPCLK, SFRA_SCI_BAUDRATE);
#endif  /* SFRA_GUI_ENABLE */
}

#if (SFRA_GUI_ENABLE == 1)
// 每个中断调一次：20 个中断 = 1ms，给串口协议做超时计数
static void Sfra_Tick(void)
{
    if (++SfraTickCnt >= 20U)
    {
        SfraTickCnt = 0U;
        if (SerialCommsTimer < 30000)
        {
            SerialCommsTimer++;
        }
    }
}
#endif  /* SFRA_GUI_ENABLE */
/*===========================================================================
 * 函数名：Sfra_Task_Background
 * 说明：主循环里调用。串口状态机在 20kHz 中断里跑，这里只剩最重的
 *       SFRA_IQ_BACKGROUND 和扫频触发/完成判定。
 *===========================================================================*/
void Sfra_Task_Background(void)
{
        // ---- SFRA 后台状态机 ----
        // 串口状态机不在主循环里跑：它已经搬进 20kHz 的 ADC 中断，
        // 每 50us 服务一次，1 字节/173us 的到达速度下物理上不会丢字节。
        // 主循环里只剩 SFRA_IQ_BACKGROUND（最重的一块），主循环慢也不会影响通讯。
        SFRA_IQ_BACKGROUND(&SFRA1);

        // ---- 自动开始扫频（SFRA_AUTO_START=1） ----
        // SfraTrig 在 .ebss 里，_cint00 启动会清零，所以不能靠"下载前置 1"；
        // 用中断计数延时，等 RefRamp 软启动(500ms)爬完再自动触发一次。
#if (SFRA_AUTO_START == 1)
        if ((SfraAutoFired == 0U) && (IsrCnt >= SFRA_AUTO_START_ISRCNT))
        {
            SfraAutoFired = 1U;
            SfraTrig      = 1;
        }
#endif

        // ---- 触发一次扫频 ----
        // 不接 GUI：在 CCS 的 Expressions 里把 SfraTrig 改成 1 即可。
        // 接 GUI 时：上位机通过 varSetBtnList[0] 置 initializationFlag=1。
        if (SfraTrig == 1)
        {
            SfraTrig = 0;
            SFRA_IQ_INIT(&SFRA1);
            SFRA1.start = 1;
            SfraDone = 0;
        }
#if (SFRA_GUI_ENABLE == 1)
        if (initializationFlag == 1)
        {
            initializationFlag = 0;
            SFRA_IQ_INIT(&SFRA1);
            SFRA1.start = 1;
            SfraDone = 0;
        }
#endif
        // 扫完判据：库会把 FreqIndex 停在 Vec_Length，start 也已由库自己清掉
        if ((SFRA1.start == 0) && (SFRA1.FreqIndex >= SFRA1.Vec_Length))
        {
            SfraDone = 1;
        }
        // 不要在这里清 SFRA1.start！Boost 里没有任何地方清它。
        // 库的 start 由 SFRA_IQ_BACKGROUND 自己管，是它判断"扫频做完没有"的状态位；
        // 外部（尤其在保护锁存时）把它清 0，会让库停在中途而 FreqIndex 还能涨。
}

#else   /* SFRA_ENABLE == 0 : 不扫频，两个函数退化成空 */

void Sfra_Task_Init(void)
{
}

void Sfra_Task_Background(void)
{
}

#endif  /* SFRA_ENABLE */
