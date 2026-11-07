/*
 * main.c
 *
 * DSP28035 单相离网逆变（全部在 CPU 上运行，不使用 CLA）
 *
 * 硬件链路：
 *   ePWM1 增减计数 TBPRD=1500 -> fsw=20kHz
 *     CTR=ZERO -> SOCA -> ADC
 *        SOC0 丢弃（ADCA0，与 SOC1 同通道，修 rev0 首采样 errata）
 *        SOC1 = ADCA0 输出电压 v_o
 *        SOC2 = ADCA1 输出电流 i_o
 *        SOC3 = ADCA2 电感电流 i_L
 *        SOC4 = ADCB1 输入电压 v_in（分压 80/3.3，只监视/保护）
 *        SOC5 = ADCB0 输入电流 i_in（放大 150 倍，只监视/保护）
 *     EOC5 -> ADCINT1 -> CPU 的 ADC_ISR
 *   ADC_ISR：换算工程量 v_o / i_o / i_L -> 查正弦表 ->
 *            LOOP_MODE=2（当前，电流单环）：反馈取**电感电流 i_L**
 *              IRef = sin*IRamp（软启动） -> PI(IRef - i_L) = V_L（电感压降，V）
 *              -> mod = (V_L + v_o滤波) / v_in滤波 -> 限幅 +-Mod_Max -> 写 CMPA
 *            LOOP_MODE=1/3/0/4 走 ADC_ISR 里对应的 #if 分支
 *   保护：纯软件（欠压 Vin<55V / 过流 |IL|>12A），见 APP/protect.c
 *         TZ 只保留
 *
 * 定点化：F28035 没有硬件 FPU，浮点全靠软件模拟，20kHz 中断里跑两个调节器
 * 很吃 CPU。本工程把控制环整体改成 TI IQmath 定点，全局格式 Q24
 * （GLOBAL_Q = 24，见 APP/IQmathLib.h）：
 *   - 所有物理量、给定、反馈、调节器内部状态都是 _iq（Q24）
 *   - 中断里的乘法一律 _IQmpy，加减法直接写（定点加减就是整数加减）
 *   - 正弦表编译期就转成 Q24 整数常量，运行期不查表乘浮点
 *   - 相位累加器用 Uint32（高 8 位表下标 + 低 24 位小数），
 *     因为 Q24 装不下 0~256 这个范围
 *   - PR 的离散系数、软启动步长、采样换算系数全部是编译期常量，
 *     整个工程运行期不再出现一次浮点运算
 *   Q24 的动态范围只有 ±128，本工程最大的运行量是 80V（电压满量程），
 *   超过 ±128 的中间量（TBPRD/2=750、2*满量程=160）在各自模块里换低 Q 处理
 *
 * 环路模式由 dcac_cfg.h 的 LOOP_MODE 选择：
 *   0 正弦开环 / 1 电压单环 / 2 电流单环 / 3 双环
 * ★ SFRA_ENABLE=1 时四种模式**全部**冻结成直流工作点（对应 TI 的 DC_CHECK），
 *   此时模式 0 就是"量受控体"那条路（固定占空比 + 注入占空比 + 采 Mod->IL）。
 * 本文件不含并网逻辑（离网模式）
 */

#include "DSP28x_Project.h"             // 项目顶层主头文件
#include "DSP2803x_GlobalPrototypes.h"  // 全局函数原型声明文件
#include "DSP2803x_EPwm_defines.h"
#include <math.h>
#include <string.h>                     /* Flash 版要用 memcpy 搬 ramfuncs */
#include "IQmathLib.h"                  // IQ 定点数学库（GLOBAL_Q 已在 dcac_cfg.h 里定为 24）
#include "dcac_cfg.h"
#include "pid.h"
#include "pr.h"
#include "spwm.h"
#include "protect.h"
#include "debug_view.h"                 // 调试镜像 / 有效值 / 示波器缓冲（Task 2 拆出去）
#include "Oled.h"                       // 0.96 寸 OLED（SSD1306，I2C 位带，GPIO28/29）
#include "oled_task.h"                  // OLED 显示任务（Task 3 拆出去）
#include "sfra_cfg.h"                   // SFRA 扫频（SFRA_ENABLE=1 时才有内容）
#include "sfra_task.h"                  // SFRA 扫频任务（Task 4 拆出去）

// 函数原型声明
void InitAdc(void);
void AdcOffsetSelfCal(void);
void Adc_Config(void);
void Adc_ZeroCal(void);
void Gpio_Config(void);
__interrupt void ADC_ISR(void);

// 正弦表 -1 ~ +1，256 点，编译期转成 Q24 整数（_IQ() 是编译期常量折叠）
// sin_table[i] = _IQ(sin(2*pi*i/256))
static const _iq sin_table[Sin_Table_Size] =
{
    _IQ(0.0000000f), _IQ(0.0245412f), _IQ(0.0490677f), _IQ(0.0735646f), _IQ(0.0980171f), _IQ(0.1224107f), _IQ(0.1467305f), _IQ(0.1709619f),
    _IQ(0.1950903f), _IQ(0.2191012f), _IQ(0.2429802f), _IQ(0.2667128f), _IQ(0.2902847f), _IQ(0.3136817f), _IQ(0.3368899f), _IQ(0.3598950f),
    _IQ(0.3826834f), _IQ(0.4052413f), _IQ(0.4275551f), _IQ(0.4496113f), _IQ(0.4713967f), _IQ(0.4928982f), _IQ(0.5141027f), _IQ(0.5349976f),
    _IQ(0.5555702f), _IQ(0.5758082f), _IQ(0.5956993f), _IQ(0.6152316f), _IQ(0.6343933f), _IQ(0.6531728f), _IQ(0.6715590f), _IQ(0.6895405f),
    _IQ(0.7071068f), _IQ(0.7242471f), _IQ(0.7409511f), _IQ(0.7572088f), _IQ(0.7730105f), _IQ(0.7883464f), _IQ(0.8032075f), _IQ(0.8175848f),
    _IQ(0.8314696f), _IQ(0.8448536f), _IQ(0.8577286f), _IQ(0.8700870f), _IQ(0.8819213f), _IQ(0.8932243f), _IQ(0.9039893f), _IQ(0.9142098f),
    _IQ(0.9238795f), _IQ(0.9329928f), _IQ(0.9415441f), _IQ(0.9495282f), _IQ(0.9569403f), _IQ(0.9637761f), _IQ(0.9700313f), _IQ(0.9757021f),
    _IQ(0.9807853f), _IQ(0.9852776f), _IQ(0.9891765f), _IQ(0.9924795f), _IQ(0.9951847f), _IQ(0.9972905f), _IQ(0.9987955f), _IQ(0.9996988f),
    _IQ(1.0000000f), _IQ(0.9996988f), _IQ(0.9987955f), _IQ(0.9972905f), _IQ(0.9951847f), _IQ(0.9924795f), _IQ(0.9891765f), _IQ(0.9852776f),
    _IQ(0.9807853f), _IQ(0.9757021f), _IQ(0.9700313f), _IQ(0.9637761f), _IQ(0.9569403f), _IQ(0.9495282f), _IQ(0.9415441f), _IQ(0.9329928f),
    _IQ(0.9238795f), _IQ(0.9142098f), _IQ(0.9039893f), _IQ(0.8932243f), _IQ(0.8819213f), _IQ(0.8700870f), _IQ(0.8577286f), _IQ(0.8448536f),
    _IQ(0.8314696f), _IQ(0.8175848f), _IQ(0.8032075f), _IQ(0.7883464f), _IQ(0.7730105f), _IQ(0.7572088f), _IQ(0.7409511f), _IQ(0.7242471f),
    _IQ(0.7071068f), _IQ(0.6895405f), _IQ(0.6715590f), _IQ(0.6531728f), _IQ(0.6343933f), _IQ(0.6152316f), _IQ(0.5956993f), _IQ(0.5758082f),
    _IQ(0.5555702f), _IQ(0.5349976f), _IQ(0.5141027f), _IQ(0.4928982f), _IQ(0.4713967f), _IQ(0.4496113f), _IQ(0.4275551f), _IQ(0.4052413f),
    _IQ(0.3826834f), _IQ(0.3598950f), _IQ(0.3368899f), _IQ(0.3136817f), _IQ(0.2902847f), _IQ(0.2667128f), _IQ(0.2429802f), _IQ(0.2191012f),
    _IQ(0.1950903f), _IQ(0.1709619f), _IQ(0.1467305f), _IQ(0.1224107f), _IQ(0.0980171f), _IQ(0.0735646f), _IQ(0.0490677f), _IQ(0.0245412f),
    _IQ(0.0000000f), _IQ(-0.0245412f), _IQ(-0.0490677f), _IQ(-0.0735646f), _IQ(-0.0980171f), _IQ(-0.1224107f), _IQ(-0.1467305f), _IQ(-0.1709619f),
    _IQ(-0.1950903f), _IQ(-0.2191012f), _IQ(-0.2429802f), _IQ(-0.2667128f), _IQ(-0.2902847f), _IQ(-0.3136817f), _IQ(-0.3368899f), _IQ(-0.3598950f),
    _IQ(-0.3826834f), _IQ(-0.4052413f), _IQ(-0.4275551f), _IQ(-0.4496113f), _IQ(-0.4713967f), _IQ(-0.4928982f), _IQ(-0.5141027f), _IQ(-0.5349976f),
    _IQ(-0.5555702f), _IQ(-0.5758082f), _IQ(-0.5956993f), _IQ(-0.6152316f), _IQ(-0.6343933f), _IQ(-0.6531728f), _IQ(-0.6715590f), _IQ(-0.6895405f),
    _IQ(-0.7071068f), _IQ(-0.7242471f), _IQ(-0.7409511f), _IQ(-0.7572088f), _IQ(-0.7730105f), _IQ(-0.7883464f), _IQ(-0.8032075f), _IQ(-0.8175848f),
    _IQ(-0.8314696f), _IQ(-0.8448536f), _IQ(-0.8577286f), _IQ(-0.8700870f), _IQ(-0.8819213f), _IQ(-0.8932243f), _IQ(-0.9039893f), _IQ(-0.9142098f),
    _IQ(-0.9238795f), _IQ(-0.9329928f), _IQ(-0.9415441f), _IQ(-0.9495282f), _IQ(-0.9569403f), _IQ(-0.9637761f), _IQ(-0.9700313f), _IQ(-0.9757021f),
    _IQ(-0.9807853f), _IQ(-0.9852776f), _IQ(-0.9891765f), _IQ(-0.9924795f), _IQ(-0.9951847f), _IQ(-0.9972905f), _IQ(-0.9987955f), _IQ(-0.9996988f),
    _IQ(-1.0000000f), _IQ(-0.9996988f), _IQ(-0.9987955f), _IQ(-0.9972905f), _IQ(-0.9951847f), _IQ(-0.9924795f), _IQ(-0.9891765f), _IQ(-0.9852776f),
    _IQ(-0.9807853f), _IQ(-0.9757021f), _IQ(-0.9700313f), _IQ(-0.9637761f), _IQ(-0.9569403f), _IQ(-0.9495282f), _IQ(-0.9415441f), _IQ(-0.9329928f),
    _IQ(-0.9238795f), _IQ(-0.9142098f), _IQ(-0.9039893f), _IQ(-0.8932243f), _IQ(-0.8819213f), _IQ(-0.8700870f), _IQ(-0.8577286f), _IQ(-0.8448536f),
    _IQ(-0.8314696f), _IQ(-0.8175848f), _IQ(-0.8032075f), _IQ(-0.7883464f), _IQ(-0.7730105f), _IQ(-0.7572088f), _IQ(-0.7409511f), _IQ(-0.7242471f),
    _IQ(-0.7071068f), _IQ(-0.6895405f), _IQ(-0.6715590f), _IQ(-0.6531728f), _IQ(-0.6343933f), _IQ(-0.6152316f), _IQ(-0.5956993f), _IQ(-0.5758082f),
    _IQ(-0.5555702f), _IQ(-0.5349976f), _IQ(-0.5141027f), _IQ(-0.4928982f), _IQ(-0.4713967f), _IQ(-0.4496113f), _IQ(-0.4275551f), _IQ(-0.4052413f),
    _IQ(-0.3826834f), _IQ(-0.3598950f), _IQ(-0.3368899f), _IQ(-0.3136817f), _IQ(-0.2902847f), _IQ(-0.2667128f), _IQ(-0.2429802f), _IQ(-0.2191012f),
    _IQ(-0.1950903f), _IQ(-0.1709619f), _IQ(-0.1467305f), _IQ(-0.1224107f), _IQ(-0.0980171f), _IQ(-0.0735646f), _IQ(-0.0490677f), _IQ(-0.0245412f)
};

// 采样与状态量（全部 Q24 定点，调试器里看到的是原始整数，
// 换算关系：显示值 = 原始整数 / 16777216，例如 1.0 = 16777216）
volatile _iq  Vout = 0;             // 输出电压 V
volatile _iq  Iout = 0;             // 输出电流 A
volatile _iq  IL   = 0;             // 电感电流 A（电流内环反馈）
volatile _iq  Vref = 0;             // 电压给定 V
volatile _iq  IRef = 0;             // 电流环给定 A（由 ISR 每拍算出，不要在这里写初值）
volatile _iq  Mod  = 0;             // 瞬时调制比
volatile Uint32 PhaseAcc = 0;       // 相位累加器：高 8 位查表下标，低 24 位 Q24 小数
volatile _iq  RefRamp = 0;          // 软启动给定峰值
volatile _iq  IRamp = 0;            // 电流环给定的软启动幅值（A），只给 LOOP_CURR_CLOSED 用

//===========================================================================
// 电流环前馈（LOOP_CURR_CLOSED）：mod = (V_L + VoutFF) * InvVin
//   V_L    : PI 输出 = 电感压降给定 L*di/dt，单位 V（不再是调制比）
//   VoutFF : 输出电压实测值，一阶 IIR 低通后当前馈用
//   VinFF  : 母线电压实测值，同样低通
//   InvVin : 1/VinFF，★ 在后台每 20ms 算一次，中断里只做乘法（除法太贵）
// ★ InvVin 必须给初值：.ebss 是 0，不清的话第一拍 mod 就是 0
//===========================================================================
volatile _iq  VoutFF = 0;                            // 输出电压前馈（滤波后）V
volatile _iq  VinFF  = _IQ(Vbus_Nominal);            // 母线电压（滤波后）V
volatile _iq  InvVin = _IQ(1.0f / Vbus_Nominal);     // 1/Vin，后台更新
volatile _iq  V_L    = 0;                            // PI 输出电压，只给 SFRA 取地址

// 欠压兜底的调制比幅值：0 -> Mod_Max，进入欠压状态时清零重爬（见 dcac_cfg.h 8c）
volatile _iq  UvRamp = 0;
// SFRA 扫频用的 0~1 工作点斜坡（RefRamp 是电压峰值 0~50.91，不能当比例用）
volatile _iq  SfraRamp = 0;

// 输入侧两路采样：只存 ADC 原始码，不参与控制环，后续留给输入过压/过流保护。
// 换算成工程量（加保护阈值时用）：
//   输入电压 V = (VinRaw - ADC_Vin_Offset) * ADC_Vin_Span / 4096
//   输入电流 A = (IinRaw - ADC_Iin_Offset) * ADC_Iin_Span / 4096
volatile _iq  Vin = 0;              // 输入电压 V（只监视）
volatile _iq  Iin = 0;              // 输入电流 A（只监视）

//===========================================================================
// ADC 零点校准结果
//   存的是"负的零点码"，也就是要加到原始码上的那个量：
//     value = (raw + OffsetNeg) * Span / 4096
//   为什么要存负号：换算乘的是 Q24 的 (raw-Offset)/4096，(raw-Offset)<<12
//   本来可能到 -3103*4096 = -1.27e7，逼近 IQmpy 的负数下界；存成 -(Offset-raw)
//   就自然变成正数偏移，乘 Span(最多 165) 也不会溢出，不需要再钳位
//   只在 main() 里赋值一次，之后全是只读，中断里读它做换算
//   初值是 0，也就是"还没校准"，换算结果等价于 (raw - 标称Offset)：
//   因为下面的换算宏用的是标准 _IQmpy，而标称值在宏里是编译期常量
//===========================================================================
volatile long VoutOffNeg = 0L;
volatile long IoutOffNeg = 0L;
volatile long ILOffNeg   = 0L;

// 有效值/平均值/示波器全部搬到 APP/debug_view.c 了。
// 这里只留"满一个工频周期"的节拍：Task 5 的 1/Vin 后台更新要挂在它上面，
// 所以它不属于调试模块。
volatile Uint16 RmsCnt    = 0U;
volatile Uint16 RmsCommit = 0U;

// 运行控制与故障
//===========================================================================
// 运行使能
// ★ 这里必须是 1U：全工程没有任何一处代码会把它写 1，
//   它一旦是 0，RefRamp 永远停在 0 -> Vref=0 -> IRef=0 -> 调制比恒 0，
//   表现就是"给定设了 4.16A，输出却只有死区那零点几伏"。
//   （17e5f6e "修改中文注释格式" 那次提交把它从 1U 改成了 0U）
//===========================================================================
volatile Uint16 Run_Enable = 1U;    // 1=允许运行（自动软启动），0=停机
volatile Uint16 FaultClear = 0U;    // 置 1 清除故障锁存
volatile Uint16 FaultLatch = FAULT_NONE;
volatile Uint16 IsrCnt = 0U;        // 中断次数，调试观察用

//===========================================================================
// 中断耗时测量：用 CpuTimer0 自由运行当时间戳，不用示波器
//   1 个计数 = 16.67 ns（60MHz）；20kHz 中断周期 = 3000 个计数 = 50us
//   换算：时间(us) = 计数值 / 60
//===========================================================================
volatile Uint32 IsrT0        = 0UL; // 本次中断入口的时间戳
volatile Uint32 IsrTLast     = 0UL; // 上一次中断入口的时间戳
volatile Uint32 IsrCycles    = 0UL; // 本次中断执行耗时（计数）
volatile Uint32 IsrCyclesMax = 0UL; // 历史最大耗时（计数）← 峰值最有用
volatile Uint32 IsrPeriod    = 0UL; // 两次中断的间隔（计数，应恒为 3000）


//===========================================================================
// 主函数入口
//===========================================================================
void main(void){
#if (Protect_Enable == 1)
    Uint16 faultHandled = 0U;       // 只有保护打开时才用它做"只封波一次"的判定
#endif

    // 系统初始化（时钟、看门狗、外设时钟）
    InitSysCtrl();

#if (SFRA_FLASH_BUILD == 1)
    // ---- Flash 运行：先把 ramfuncs 从 Flash 搬到 RAM ----
    // 顺序不能反：InitFlash() 自己在 ramfuncs 段里，没搬之前它在 Flash 里跑，
    // 而那时 Flash 等待周期还没设好，会取到错指令。
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart,
           (size_t)&RamfuncsLoadEnd - (size_t)&RamfuncsLoadStart);
#endif
    // InitFlash() 无条件调用：
    //   RAM 版里 ramfuncs 是 LOAD=RUN=SARAM，调它没有副作用；
    //   但能挡住「换了 Flash 脚本却忘了开 SFRA_FLASH_BUILD」这种静默失败
    //   ——那样会从 Flash 取指却用复位默认等待周期，20kHz 中断根本跑不完。
    InitFlash();

    // 中断系统初始化
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    // 外设初始化
    // 顺序有讲究：
    //   InitAdc()          上电 ADC、拿 OTP 里的校准值
    //   AdcOffsetSelfCal() 校 ADC 内核自己的失调（内部 VREFLO）
    //   Adc_ZeroCal()      软件校零，量整条调理链路的零点（要在内核失调修好之后）
    //   Adc_Config()       正式配置 SOC 通道和触发源，并把轮询指针拨回 SOC0
    // Adc_ZeroCal 必须在 SPWM_Init 之前：那时 EPWM 还没起振，不会有 SOCA
    // 来抢轮询指针，测出来的零点才是干净的
    InitAdc();
    AdcOffsetSelfCal();
    Adc_ZeroCal();
    Adc_Config();

    // 调试镜像：清零累加器和滤波器状态（ADC 校零之后调用，校零结果已经交出去）
    Debug_View_Init();

    Oled_Task_Init();

    SPWM_Init();
    Gpio_Config();

    Sfra_Task_Init();

    // 保护：只配 TZ 的软件封波通路，不碰 COMP/Digital Compare
    Protect_Init();

    // 中断向量绑定（只有 ADCINT1；TZ 中断已经不用了）
    EALLOW;
    PieVectTable.ADCINT1     = &ADC_ISR;            // ADC 中断：控制环
    EDIS;

    // 初始化 PR：电压外环，准谐振 w0 = 314.159, wc = 10
    // r = w0*Ts/2、q = wc*Ts/2 已经在 dcac_cfg.h 里按编译期常量算成 Q24
    // （w0 = 314 超出 Q24 的动态范围，所以在设计期就折进 r、q，不进运行期），
    // 因此初始化阶段也全是定点运算
    // ★ 电流单环不需要 PR，pr.c 里整个函数都编掉了，这里也要跟着不调用
#if (LOOP_MODE != LOOP_CURR_CLOSED)
    PR_Design(&PR_V, KP_V, KR_V, KI_V,
              R_V, Q_V);
#endif
    PR_V.Ref = 0;
    PR_V.Fdb = 0;
    PR_Reset(&PR_V);
    // 限幅必须是 Q24！原来这里写的是裸小数 0.95，赋给 _iq 会被截断成 0，
    // __clamp(y, 0, 0) 让 PR 输出恒等于 0，表现出来就是"闭环没输出"。
    // 现在 PR 输出直接当调制比用，所以限到 ±Mod_Max
    PR_V.OutMin = -Mod_Max;
    PR_V.OutMax =  Mod_Max;

    // 初始化 PI_I
    // ★ 量纲随模式不同：
    //   LOOP_CURR_CLOSED：前馈结构，PI 输出是电压（V）。
    //     下面给的 ±51V 只是上电初值，ISR 每拍会按真实电压余量重设
    //     （而且是**不对称**的区间，见 ADC_ISR 里的动态限幅）
    //   其它模式        ：PI 输出直接当调制比，限幅 ±Mod_Max
    PI_I.Ref = 0;
    PI_I.Fdb = 0;
    PI_I.Integral = 0;
    // ★ 误差前置限幅：p_term = Kp*err 必须留在 Q24 的 ±128 内。
    //   Kp=11.74 时上限只有 10.9A，而过流阈值是 12A —— 那一段既没保护、又会
    //   把 p_term 算翻符号（+128 -> -128），PI 反向打，现场就是"跑飞"。
    //   取 Iref_Max(8A)，留 30% 余量。
    PI_I.ErrMax = Iref_Max;
#if (LOOP_MODE == LOOP_CURR_CLOSED)
    PI_I.Kp     = KP_I;                 // V/A
    PI_I.KiTs   = KI_I_Ts;
    PI_SetLimits(&PI_I, -V_L_Max, V_L_Max);   // 固定宽限幅（1.0 pu of Vbus），一行
#else
    PI_I.Kp     = KP_I3;                // mod/A
    PI_I.KiTs   = KI_I3_Ts;
    PI_SetLimits(&PI_I, -Mod_Max, Mod_Max);   // mod/A，直接限调制比，一行
#endif

    // ---- CpuTimer0 配成自由运行的周期计数器，给 ADC_ISR 量耗时 ----
    //   PRD 拉满 -> 从 0xFFFFFFFF 往下数，60MHz 下 71.6s 才绕回，够用
    //   TSS=1 先停，写 TRB=1 把计数器重装成 PRD，再 TSS=0 放它跑
    //   注意：TIM 是"往下数"的，所以耗时 = 入口时间戳 - 当前时间戳
    CpuTimer0Regs.TCR.bit.TSS = 1;              // 先停
    CpuTimer0Regs.TPR.all     = 0;              // 不分频（1 计数 = 1 个 SYSCLKOUT）
    CpuTimer0Regs.TPRH.all    = 0;
    CpuTimer0Regs.PRD.all     = 0xFFFFFFFFUL;   // 周期拉满 = 自由运行
    CpuTimer0Regs.TCR.bit.TRB = 1;              // 计数器重装
    CpuTimer0Regs.TCR.bit.TSS = 0;              // 启动

    // 开 PIE 和 CPU 中断（只有 ADCINT1 一路；TZ 中断已经不用了）
    EALLOW;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;      // ADCINT1
    IER |= M_INT1;
    PieCtrlRegs.PIEIER2.bit.INTx1 = 0;
    IER &= (Uint16)(~M_INT2);
    EDIS;

    EINT;
    ERTM;

    //=======================================================================
    //  后台死循环：本工程只处理故障复位
    //  运行状态全部在 ADC_ISR 里完成
    //=======================================================================
    for (;;)
    {
        Sfra_Task_Background();

#if (Protect_Enable == 1)
        // 过流锁存才真正封波；欠压不停机（切兜底），所以这里只认 OC。
        // Protect_Enable = 0 时这段不编译 —— Protect_ForceStop() 本身是无条件生效的
        // 主动急停，不在这里挡住的话，只置标志也会把桥臂关掉
        if (FaultLatch == FAULT_OC)
        {
            if (faultHandled == 0U)
            {
                faultHandled = 1U;
                Protect_ForceStop();
                RefRamp = 0;
                IRamp   = 0;
                UvRamp  = 0;
                PhaseAcc = 0U;
            }
        }
        else
        {
            faultHandled = 0U;
        }
#endif

//        // 复位故障：把 FaultClear 置 1（调试器或以后加的通信）
//        if (FaultClear != 0U)
//        {
//            FaultClear = 0U;
//            Protect_Clear();            // 清过流锁存 + 清 TZ 触发
//            FaultLatch = FAULT_NONE;
//#if (Protect_Enable == 1)
//            faultHandled = 0U;
//#endif
//
//            // 清两个环的历史，重新软启动
//            PR_Reset(&PR_V);
//            PI_I.Integral = 0;
//            RefRamp = 0;
//            IRamp   = 0;
//            UvRamp  = 0;
//            PhaseAcc = 0U;
//        }

        // OLED 刷新：函数内部自己按 500ms 节流，所以每圈都可以调
        // 刷一屏要几十毫秒（I2C 位带，没有硬件外设），所以不能刷太快
#if (SFRA_ENABLE == 0)
        Oled_Task_Run(IsrCnt, FaultLatch);
#endif

        // 后台：满一个工频周期（20ms）做一次
        //   1) 重算 1/Vin（前馈用。母线是毫秒级变化，20ms 更新足够，
        //      顺带把 64/32 除法挪出 20kHz 中断）
        //   2) 刷调试镜像的有效值（debug_view.c）
#if (DEBUG_VIEW_ENABLE== 0)
        if (RmsCommit != 0U)
        {
            RmsCommit = 0U;

            {
                _iq vs = VinFF;

                if (vs < Vin_Ff_Min) { vs = Vin_Ff_Min; }   // Review Focus 1
                if (vs > Vin_Ff_Max) { vs = Vin_Ff_Max; }   // Review Focus 2

                InvVin = _IQdiv(_IQ(1.0f), vs);
            }

            Debug_View_Background();
        }
#endif
    }
}


//===========================================================================
// ADC 中断服务函数：采样完成后执行完整控制环，20kHz
//   整个中断里没有一次浮点运算，全部是 Q24 定点加减和 _IQmpy
//===========================================================================
#if (SFRA_FLASH_BUILD == 1)
// Flash 版：20kHz 中断必须从 RAM 取指。Flash 有等待周期，
// 中断跑在 Flash 里时 50us 预算会被吃掉，后台串口任务饿死 ->
// SFRA GUI 一点 Start Sweep 就「连接丢失」。
#pragma CODE_SECTION(ADC_ISR, "ramfuncs")
#endif
__interrupt void ADC_ISR(void)
{
    _iq sin_val;

    // ---- 0. 时间戳：量中断耗时（不用示波器）----
    //   TIM 往下计数，耗时 = 入口时间戳 - 出口时间戳
    //   IsrPeriod 应恒等于 3000（3000/60MHz = 50us），对不上就说明中断不是 20kHz
    IsrT0     = CpuTimer0Regs.TIM.all;
    IsrPeriod = IsrTLast - IsrT0;
    IsrTLast  = IsrT0;

    GpioDataRegs.GPASET.bit.GPIO5 = 1;      // 中断计时脚拉高

    // ---- 1. 采样并换算工程量 ----
    // 固定槽位：SOCn 的结果就在 ADCRESULTn 里，直接按下标取，不做任何判断
    //   ADC_Ch_Dummy -> SOC0/ADCRESULT0 丢样（ADCA0 的第一次采样不准）
    //   ADC_Ch_Vout  -> SOC1/ADCRESULT1 输出电压  ← 与 SOC0 同通道，用这一次
    //   ADC_Ch_Iout  -> SOC2/ADCRESULT2 输出电流
    //   ADC_Ch_IL    -> SOC3/ADCRESULT3 电感电流
    //   ADC_Ch_Vin   -> SOC4/ADCRESULT4 输入电压（只监视）
    //   ADC_Ch_Iin   -> SOC5/ADCRESULT5 输入电流（只监视）
    // 每个通道的零点码和满码量程在 dcac_cfg.h 里按调理电路标定，
    // 换算宏展开成 __IQmpy((raw-零点码)<<12, 量程(Q17), 17)
    // 用 ADC_To_IQ_Cal：零点码用实测校零结果（主程序启动时测出来的），
    // 没校准或该校通道被一致性检查挡掉时 OffsetNeg = 0，
    // 此时它和 ADC_To_IQ(标称零点码) 完全等价
    Vout = ADC_To_IQ_Cal(AdcResult.ADCRESULT1, ADC_Vout_Offset, VoutOffNeg, ADC_Vout_Span);
    Iout = ADC_To_IQ_Cal(AdcResult.ADCRESULT2, ADC_Iout_Offset, IoutOffNeg, ADC_Iout_Span);
    IL   = ADC_To_IQ_Cal(AdcResult.ADCRESULT3, ADC_IL_Offset,   ILOffNeg,   ADC_IL_Span);

    // 输入电压（ADCB1）/ 输入电流（ADCB0）：只存原始码，不换算、不参与控制环。
    // 中断里一个乘法都不多做，后面加输入保护时再按 dcac_cfg.h 里的
    // ADC_Vin_*/ADC_Iin_* 换算，或者直接拿码值和阈值码值比较
    // 输入侧两路（直流，只监视/保护）：不校零，OffsetNeg 固定为 0
    Vin = ADC_To_IQ_Cal(AdcResult.ADCRESULT4, ADC_Vin_Offset, 0L, ADC_Vin_Span);
    Iin = ADC_To_IQ_Cal(AdcResult.ADCRESULT5, ADC_Iin_Offset, 0L, ADC_Iin_Span);

    // ---- 1b. 有效值累加 + 示波器缓冲（实现在 debug_view.c）----
    //   累加是纯定点整数运算；示波器缓冲受 DEBUG_VIEW_ENABLE 控制，关掉时是空函数
    Debug_View_IsrUpdate(Vout, Iout, IL, Vin, Iin, Mod);
    Debug_View_ScopeSample(Iout, IL);

    // 满一个工频周期（400 拍 = 20ms）置一次提交标志，后台处理
    if (++RmsCnt >= Rms_Cycle_Samples)
    {
        RmsCnt = 0U;
        RmsCommit = 1U;
    }

#if (Debug_Fixed_Mod_En == 1)

    // 固定调制比调试模式：控制环完全不跑，CMPA 已经在 SPWM_Init 里设成固定值
    // 只验证 ePWM + 死区 + 驱动这一段，连 ADC 结果都不参与

#else

    // ---- 2. 软件保护判断（欠压 UV + 电感过流 OC）----
    // Protect_Enable = 0 时照样算、照样置标志，但下面不停机也不封波，
    // 方便先在 Expressions 里看 UvFault/OcFault 真不真，再决定要不要打开
    Protect_Update(Vin, IL);

    if (Protect_OcFault != 0U)
    {
        FaultLatch = FAULT_OC;          // 锁存：要 FaultClear 才清
    }
    else if (Protect_UvFault != 0U)
    {
        FaultLatch = FAULT_UV;          // 不锁存：电压回来自动清
    }
    else
    {
        FaultLatch = FAULT_NONE;
    }

    // 只有过流（且保护已打开）才停机；欠压是切兜底，不是停机
    if ((Protect_Enable == 1U) && (FaultLatch == FAULT_OC))
    {
        // 故障：清历史、输出中点
        PR_Reset(&PR_V);
        PI_I.Integral = 0;
        RefRamp = 0;
        IRamp   = 0;
        UvRamp  = 0;
        Mod = 0;
        SPWM_SetNeutral();
    }
    else
    {
        // ---- 3. 软启动：给定峰值从 0 爬到额定 ----
        // 每拍加一个 Q24 步长，定点加法累加不会引入额外误差
        if (Run_Enable != 0U)
        {
            RefRamp += Soft_Start_Step;
            if (RefRamp > Vref_Peak)
            {
                RefRamp = Vref_Peak;
            }
        }
        else
        {
            RefRamp = 0;
        }

        // SFRA 工作点的 0~1 斜坡：和 RefRamp 同一个软启动时长
        if (Run_Enable != 0U)
        {
            SfraRamp += Sfra_Ramp_Step;
            if (SfraRamp > _IQ(1.0f))
            {
                SfraRamp = _IQ(1.0f);
            }
        }
        else
        {
            SfraRamp = 0;
        }

        // ---- 4. 相位累加 + 查表：phase_step = 256*50/20000 = 0.64 ----
        // PhaseAcc 高 8 位是表下标、低 24 位是 Q24 小数，
        // 与 0xFFFFFFFF 相与即对 256 取模，等价于原来的 Phase -= 256
        PhaseAcc = (PhaseAcc + Phase_Step_Q24) & Phase_Mask;
        sin_val = sin_table[(Uint16)(PhaseAcc >> Phase_Acc_Bits)];
        Vref = _IQmpy(sin_val, RefRamp);

#if (SFRA_ENABLE == 1)
        // ---- SFRA 扫频版：**所有模式**都把参考冻结成直流工作点 ----
        // 对应 TI 例程的 DC_CHECK=1。正弦参考没有稳态工作点，环路一直在追
        // 移动目标，工频基波会把注入的小扰动淹掉，扫出来是噪声。
        //   模式 0    -> 固定占空比（注入点=占空比，采 Mod->IL = 受控体本身）
        //   模式 2    -> 直流电流给定
        //   模式 1/3  -> 直流电压给定
        // ★ 工作点必须跟软启动一起爬（否则复位后第一个中断就把功率级顶起来，
        //   绕过了 Run_Enable 和软启动）。
        // ★★ 这里要用 0~1 的斜坡 SfraRamp，**不能**用 RefRamp：
        //     RefRamp 是"电压峰值"，爬到 Vref_Peak = 50.91，不是 0~1。
        //     之前拿它当比例乘，工作点变成 1A×50.91 = 50.9A，
        //     注入的 ±0.3A 随后被 Iref_Max(8A) 限幅整个吃掉，
        //     SFRA 量到的是纯噪声 —— 表现就是幅频一条平线、相位乱跳 ±180°。
        {
            _iq op = (LOOP_MODE == LOOP_SINE_OPEN)   ? SFRA_DUTY_DC
                   : (LOOP_MODE == LOOP_CURR_CLOSED) ? SFRA_IREF_DC
                   : SFRA_VREF_DC;
            Vref = _IQmpy(op, SfraRamp);        // SfraRamp: 0 -> 1.0
        }
#endif

#if (LOOP_MODE == LOOP_CURR_CLOSED)
        // ---- 电流单环：电压环旁路，注入点在电流给定 ----
        // ★ 这个分支里不能再调用 SFRA_INJ 于电压给定，否则一个中断注入两次，
        //   库内部相位每个采样推进两步，实测频率会是标签值的 2 倍。
        PR_Reset(&PR_V);
#if (SFRA_ENABLE == 1)
        // ★ 先给直流工作点限幅，再注入：顺序反过来（先注入后限幅）的话
        //   只要工作点接近 Iref_Max，±0.3A 的扰动就会被整个削平，扫出来全是噪声
        IRef = Vref;
        if (IRef > Iref_Max)
        {
            IRef = Iref_Max;
        }
        else if (IRef < -Iref_Max)
        {
            IRef = -Iref_Max;
        }
        IRef = SFRA_INJ(IRef);
#else
        // ★ 电流给定 = 安培，不是电压环的 Vref（伏特）。
        //   Vref = sin_val*RefRamp，幅值会爬到 Vref_Peak = 50.91，
        //   直接当电流给定就是 50.9A，PI 一拍就顶到 ±Mod_Max。
        //   这里用电流环自己的软启动幅值 IRamp（0 -> Iref_Peak），
        //   Iref_Peak = sqrt(2)*4.16 = 5.883A（4.16Arms/50Hz）。
        if (Run_Enable != 0U)
        {
            IRamp += Iref_Ref_Step;
            if (IRamp > Iref_Peak)
            {
                IRamp = Iref_Peak;
            }
        }
        else
        {
            IRamp = 0;
        }

        IRef = _IQmpy(sin_val, IRamp);      // 50Hz 正弦电流给定（幅值 IRamp）
#endif
        // 给定限幅（一行）。Iref_Max 原来只在 dcac_cfg.h 里定义、从没被用过
        IRef = PI_CLAMP(IRef, -Iref_Max, Iref_Max);

        PI_I.Ref = IRef;
        PI_I.Fdb = IL;

        // ---- 前馈：mod = (V_L + VoutFF) * (1/Vin) ----
        // 结构照 TI 逆变器例程式 (12)：invDutyPU = (Gi + invVoltInst)/invVbusInst
        //   分子 = 电感压降给定 + 输出电压（都是电压量纲）
        //   分母 = 母线电压，用后台算好的倒数，中断里只做乘法
        // 先把 Vout / Vin 滤波：开关纹波和母线 100Hz 脉动不能直接灌进占空比
        VoutFF += (Vout - VoutFF) >> Vff_Shift;
        VinFF  += (Vin  - VinFF)  >> Vff_Shift;

        // ---- 限幅：固定宽限幅（TI 风格），一行 ----
        // 补偿器自己的边界给 1.0 pu of Vbus（V_L_Max = 60V），真正的限幅交给末级 Mod_Max。
        // 好处：不用每拍按 Vout 重算，也不可能出现 OutMax < OutMin 的乱序边界。
        // 想切回"动态余量限幅"就把下面这一行换成注释里那两行：
        //   PI_SetLimits(&PI_I, -_IQmpy(Mod_Max,VinFF) - VoutFF,
        //                       +_IQmpy(Mod_Max,VinFF) - VoutFF);
        PI_SetLimits(&PI_I, -V_L_Max, V_L_Max);

        V_L = PI_Calc(&PI_I);                       // Q24，单位 V

        Mod = _IQmpy(V_L + VoutFF, InvVin);         // 无量纲调制比

#if (Protect_Enable == 1)
        // ---- 欠压兜底：切固定 Mod_Max 幅值正弦，覆盖上面算出来的电流环输出 ----
        // 幅值走自己的斜坡 UvRamp，Run_Enable 门控；
        // 退出时清零，下次进入重新从 0 爬（Review Focus 4）
        if (Protect_UvFault != 0U)
        {
            if (Run_Enable != 0U)
            {
                UvRamp += Uv_Ref_Step;
                if (UvRamp > Mod_Max)
                {
                    UvRamp = Mod_Max;
                }
            }
            else
            {
                UvRamp = 0;
            }

            Mod = _IQmpy(sin_val, UvRamp);
            PI_I.Integral = 0;      // 不让闭环历史带进来
            V_L = 0;
            IRef = 0;
            IRamp = 0;              // 回闭环时电流给定从 0 重爬，不能一步跳回 5.88A
        }
        else
        {
            UvRamp = 0;
        }
#endif

#elif (LOOP_MODE == LOOP_VOLT_CLOSED)
        // ---- 电压单环：电流环旁路，PR 输出直接当调制比 ----
        // （PR 输出的量纲本来是"电流给定"，单环时直接当调制比用，
        //   增益靠 KP_V/KR_V 调，和之前跑单电压环时一样）
#if (SFRA_ENABLE == 1)
        PR_V.Ref = SFRA_INJ(Vref);
#else
        PR_V.Ref = Vref;
#endif
        PR_V.Fdb = Vout;
        IRef    = PR_Calc(&PR_V);           // 这个量就是受控体输入
        Mod     = IRef;
        PI_I.Integral = 0;

#elif (LOOP_MODE == LOOP_DUAL)
        // ---- 双环：PR 给电流给定，PI 给调制比 ----
#if (SFRA_ENABLE == 1)
        PR_V.Ref = SFRA_INJ(Vref);
#else
        PR_V.Ref = Vref;
#endif
        PR_V.Fdb = Vout;
        IRef = PR_Calc(&PR_V);              // PR 的输出 = 电流环给定 = 受控体输入

        PI_I.Ref = IRef;
        PI_I.Fdb = IL;
        Mod = PI_Calc(&PI_I);
#else
        // ---- 模式 0：正弦开环 + 软启动 ----
#if (SFRA_ENABLE == 1)
        // 扫频版：固定占空比直流工作点。注入点在占空比，采集 Mod->IL，
        // 量到的就是**受控体本身**（TI 例程 BUILD1 + DC_CHECK=1 就是这条）
        Mod  = SFRA_INJ(Vref);          // Vref = SFRA_DUTY_DC * SfraRamp
        IRef = 0;
        PR_Reset(&PR_V);
        PI_I.Integral = 0;
        SFRA_COL(&Mod, &IL);            // 采集也在这里面做，别在下面重复调
#else
        // 正常运行：瞬时调制比直接来自正弦表，不走 PR/PI
        Mod = _IQmpy(sin_val, Mod_Amp_Open);
        IRef = 0;
        PR_Reset(&PR_V);
        PI_I.Integral = 0;
#endif
#endif

#if (SFRA_ENABLE == 1) && (LOOP_MODE != LOOP_SINE_OPEN)
        // ---- 采集：第一个参数是"受控体的输入"，第二个是"受控体的输出" ----
        // 参考量 r 由 SFRA_IQ_INJECT 自己记着，所以电压环这里要传 PR 的输出 IRef，
        // 不能传 PR_V.Ref（那等于 u = r，算出来的是闭环增益 Y/R，不是受控体）
        // 每个中断只能调用一次 INJECT / 一次 COLLECT
#if (LOOP_MODE == LOOP_CURR_CLOSED)
        // ★ 前馈之后受控体的输入是"桥臂电压指令" V_L，不是占空比：
        //   duty 还要除母线、加输出电压前馈，拿 duty 当输入量出来的是另一条传函。
        //   TI 例程同样采补偿器输出（SFRA_F_COLLECT(&cntl3p3z_InvI_vars.Out, ...)）
        SFRA_COL(&V_L, &IL);
#else
        SFRA_COL(&IRef, &Vout);
#endif
#endif

        // ---- 7. 限幅并写 CMPA（Mod 仍保留在全局里给调试/SFRA 用）----
        Mod = PI_CLAMP(Mod, -Mod_Max, Mod_Max);
        SPWM_SetMod(Mod);
    }

#endif

    IsrCnt++;

#if (SFRA_ENABLE == 1) && (SFRA_GUI_ENABLE == 1)
    Sfra_Tick();        // 20 个中断 = 1ms，给串口协议做超时
    SerialHostComms();  // 串口状态机放中断里：每 50us 服务一次，不会丢字节
#endif  /* 不接 SFRA GUI 时这里不占任何时间 */

    // ---- 8. 清中断标志 ----
    // INT1CONT = 1（连续模式）时标志由硬件在响应中断时自动清掉，
    // 这里再写一次是为了清掉可能残留的旧标志，和连续模式不冲突。
    // 溢出标志也清一下：它本身不产生中断，但留着会掩盖真正的丢拍
    AdcRegs.ADCINTFLGCLR.all = 0x0001;
    AdcRegs.ADCINTOVFCLR.all = 0x0001;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    // ---- 9. 中断耗时（CPU 周期数，60MHz 下 1 计数 = 16.67ns）----
    //   IsrCycles    = 本次耗时      -> 时间(us) = IsrCycles / 60
    //   IsrCyclesMax = 历史峰值      <- 看最坏情况用这个
    {
        Uint32 dt = IsrT0 - CpuTimer0Regs.TIM.all;

        IsrCycles = dt;
        if (dt > IsrCyclesMax)
        {
            IsrCyclesMax = dt;
        }
    }

    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;    // 中断计时脚拉低
}

//===========================================================================
// ADC 零点校准（软件校零）
//
// 做什么：
//   把 SOC0~SOC5 的触发源临时改成"软件触发"，逐个通道强制转换并累加
//   ADC_ZeroCal_Samples 次，求平均得到实测零点码，再折成 OffsetNeg 存起来。
//   跑完把触发源还给 EPWM1 SOCA，并把轮询指针拨回 SOC0。
//
// 放在初始化序列的哪一步：
//   InitAdc() -> AdcOffsetSelfCal() -> Adc_ZeroCal() -> Adc_Config() -> SPWM_Init()
//   必须在 AdcOffsetSelfCal() 之后（内核失调已经修好，测出来的才准），
//   在 SPWM_Init() 之前（EPWM 还没起振，不会有 SOCA 来抢轮询指针），
//   也在 EINT 之前（此时没有任何中断，不会被 ISR 打断）。
//
// 前提：调用这一刻各路输入必须真的是零，否则校出来的是"当时的输入"，
//       不是零点。见 dcac_cfg.h 第 5b 节的说明。不满足就把
//       ADC_ZeroCal_Enable 置 0。
//
// 保护：每路用前 ADC_ZeroCal_SpreadN 点算 (max-min)，
//       超过 ADC_ZeroCal_MaxSpread 就认为这一路悬空/在跳，
//       放弃校准、OffsetNeg 留 0（等价于用标称 Offset），并累加 CalMiss。
//       通道之间的失调差异各算各的，互不影响。
//===========================================================================
void Adc_ZeroCal(void)
{
#if (ADC_ZeroCal_Enable == 1)
    Uint16 ch;
    Uint16 i;
    Uint16 raw;
    Uint32 sum;
    long   zeroCode;
    long   nominal;
    long   offneg;
    float  CalPinVo = 0.0f;     // 实测引脚电压，最后一次性交给 debug_view
    float  CalPinIo = 0.0f;
    float  CalPinIl = 0.0f;

    EALLOW;
    AdcRegs.ADCINTFLGCLR.all = 0x0001;      // 清掉可能残留的中断标志
    AdcRegs.ADCINTOVFCLR.all = 0x0001;

    // 全部 SOC 改成软件触发（TRIGSEL=0，靠 ADCSOCFRC1 强制启动）
    // SOC0~SOC5 后面 Adc_Config() 会统一恢复成 EPWM1 SOCA，
    // 这里只是临时借用一下，好在上电阶段把零点测出来
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL = 0;

    // 松开轮询指针，从 SOC0 开始
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 1;
    while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 1) {}
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0;
    while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 0) {}

    // 只校 Vout / Iout / IL 三路。
    // 输入侧 Vin/Iin 不校零：它们是绝对值通道，上电时输入本来就不是 0
    // （母线已经带电），当零点扣掉的话运行时会少算一整段输入电压/电流
    // （实测 35V 被扣成 1.9V）。
    for (ch = 0U; ch < 3U; ch++)
    {
        // 把待测通道放到 SOC0（轮询指针重新拨到 SOC0 之后第一个转的就是它）
        // 通道号直接取 dcac_cfg.h 的 ADC_Ch_* 宏，不在这里另抄一份：
        // 以后再改通道映射，只改 dcac_cfg.h 一处，这里自动跟着走，
        // 不会出现"映射改了、校零还校在旧通道上"这种错位
        switch (ch)
        {
            case 0U: AdcRegs.ADCSOC0CTL.bit.CHSEL = ADC_Ch_Vout; break;
            case 1U: AdcRegs.ADCSOC0CTL.bit.CHSEL = ADC_Ch_Iout; break;
            default: AdcRegs.ADCSOC0CTL.bit.CHSEL = ADC_Ch_IL;   break;
        }

        sum = 0UL;

        // 换通道后先空转一次并丢弃：采样保持电容和结果寄存器里都还是上一路的
        // 电压，换通道后的第一次转换结果是上一路的旧值（实测会偏 1133 码）。
        for (i = 0U; i <= ADC_ZeroCal_Samples; i++)
        {
            Uint16 d;

            // 轮询指针拨回 SOC0，保证这一拍转的确实是我们要测的那个通道
            AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 1;
            while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 1) {}
            AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0;
            while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 0) {}

            AdcRegs.ADCSOCFRC1.all = 0x0001;        // 强制 SOC0 转换

            // SOCFLG 在转换"开始"时就清零了，不是转换结束。
            // 所以要再等一个转换时间（13 个 ADCCLK 约 220ns，这里给约 1us）
            // 再读结果，否则读到的是上一次转换留在结果寄存器里的值。
            while (AdcRegs.ADCSOCFLG1.bit.SOC0 != 0U) {}
            for (d = 0U; d < 60U; d++) { ; }

            raw = AdcResult.ADCRESULT0;

            if (i == 0U)
            {
                continue;           // 丢掉换通道后的第一次
            }

            sum += (Uint32)raw;
        }

        zeroCode = (long)(sum >> ADC_ZeroCal_Shift);

        // 该通道零点码的标称值（编译期常量，和上面 CHSEL 的顺序一一对应）
        switch (ch)
        {
            case 0U: nominal = (long)ADC_Vout_Offset; break;
            case 1U: nominal = (long)ADC_Iout_Offset; break;
            default: nominal = (long)ADC_IL_Offset;   break;
        }

        // 只校 Vout / Iout / IL 三路（它们有直流偏置，零点是明确的）
        offneg = nominal - zeroCode;    // offneg = nominal - raw_zero

        switch (ch)
        {
            case 0U:
                VoutOffNeg = offneg;    // ADCA0 输出电压零点
                CalPinVo = (float)zeroCode * ADC_Vref / 4096.0f;
                break;
            case 1U:
                IoutOffNeg = offneg;    // ADCA1 输出电流零点
                CalPinIo = (float)zeroCode * ADC_Vref / 4096.0f;
                break;
            default:
                ILOffNeg   = offneg;    // ADCA2 电感电流零点
                CalPinIl = (float)zeroCode * ADC_Vref / 4096.0f;
                break;
        }
    }

    // 把实测的引脚电压交给调试镜像（OLED 的 Z: 显示用）
    Debug_View_SetCalPin(CalPinVo, CalPinIo, CalPinIl);

    // 恢复：轮询指针拨回 SOC0，SOC 挂起标志清掉
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 1;
    while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 1) {}
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0;
    while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 0) {}
    AdcRegs.ADCSOCFLG1.all = 0xFFFF;
    AdcRegs.ADCINTFLGCLR.all = 0x0001;
    AdcRegs.ADCINTOVFCLR.all = 0x0001;

    EDIS;
#endif
}

//===========================================================================
// ADC 配置函数
//   SOC0 -> ADCA0，第一次采样按原设计丢弃（与 SOC1 同通道）
//   SOC1 -> ADCA0 输出电压
//   SOC2 -> ADCA1 输出电流
//   SOC3 -> ADCA2 电感电流
//   SOC4 -> ADCB1 输入电压（只监视）
//   SOC5 -> ADCB0 输入电流（只监视）
//   TRIGSEL=5 : EPWM1 SOCA（CTR=ZERO）
//   INT1SEL=5 : EOC5 触发 ADCINT1
//   INTPULSEPOS=1 : 转换结束结果锁存后才发中断脉冲
//
// 【为什么 INT1CONT 必须写 1】—— 这里踩过的坑，别改回去
//   INTSEL1N2.INT1CONT = 0（关闭连续模式）时，中断标志 ADCINTFLG.ADCINT1
//   是"锁存"的：EOC 到来时如果标志还是 1，ADC 只置一个溢出位 ADCINTOVF，
//   不再产生新的中断脉冲。于是只要有一个脉冲没被及时清掉（ISR 执行时间
//   接近或超过一个载波周期 50us、进 ISR 时被别的事情挡住、清标志的时机
//   不对……），之后每一拍的 EOC 都会被"吃掉"，表现就是中断进了一次就
//   再也不进，IsrCnt 永远停在 1。
//   TI 对这个 ADCINTFLG 阻塞问题的官方修法就是文档里那句
//   "Use Continue-to-Interrupt Mode to prevent the ADCINTFLG from blocking
//    additional ADC interrupts"，也就是 INT1CONT = 1：
//   连续模式下标志由硬件在 ISR 被响应时自动清掉，不会再锁死。
//   ISR 里的 ADCINTFLGCLR 保留（清掉初始化阶段可能残留的旧标志），
//   和连续模式不冲突。
//===========================================================================
void Adc_Config(void)
{
    EALLOW;

    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;    // 结果锁存之后再发中断脉冲

    AdcRegs.INTSEL1N2.bit.INT1E     = 1;    // 使能 ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 1;    // 连续模式！见上面那段说明，不能写 0
    AdcRegs.INTSEL1N2.bit.INT1SEL   = ADC_EOC_For_INT1;  // EOC5 触发 ADCINT1

    // SOC0 丢一次（ADCA0 = 输出电压，与 SOC1 配成同通道连采两次）
    // 这个配对必须相邻，中间不能插别的通道，否则 rev0 首采样勘误修法失效
    AdcRegs.ADCSOC0CTL.bit.CHSEL    = ADC_Ch_Dummy;
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = ADC_Ch_Vout;
    AdcRegs.ADCSOC2CTL.bit.CHSEL    = ADC_Ch_Iout;
    AdcRegs.ADCSOC3CTL.bit.CHSEL    = ADC_Ch_IL;
    AdcRegs.ADCSOC4CTL.bit.CHSEL    = ADC_Ch_Vin;
    AdcRegs.ADCSOC5CTL.bit.CHSEL    = ADC_Ch_Iin;

    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;    // EPWM1 SOCA
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL  = 5;

    // 采样窗 = (ACQPS+1) 个 ADCCLK。ADCCLK = 60MHz -> 1 个 ADCCLK = 16.67ns
    // 6 -> 7 个 = 117ns；6 路背靠背总共 800ns，全部落在 CTR=ZERO 前后的安静区。
    // ★ 不要为了"S+H 充得更充分"把它调大：6 路是串行转换的，ACQPS=20 会让
    //   最后一路拖到触发后 2.1us 才采，CMPA 很小时那正是开关沿附近，
    //   反而把开关噪声采进来（实测电流通道毛刺变大）。
    //   源阻抗低时 117ns 已经够：117ns/(9*1.64pF) 允许源阻抗约 8kohm。
    //   调理电路噪声大要靠 RC 和布线解决，不是靠加长采样窗。
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 6;  //Vout
    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 6;  //Iout
    AdcRegs.ADCSOC3CTL.bit.ACQPS    = 6;  //IL
    AdcRegs.ADCSOC4CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC5CTL.bit.ACQPS    = 6;

    // ---- 把 SOC 轮询指针拨回 SOC0，并清掉自校准留下的旧标志 ----
    // InitAdc() 之后的 AdcOffsetSelfCal() 会把 16 个 SOC 全改成 B5 通道
    // 并强制转换 256 次，跑完以后轮询指针和 SOC 标志都是"脏"的。
    // SOCPRICTL.SOCPRIORITY 写 1 再写回 0 会让硬件复位轮询指针
    // （写 1 -> 等到读回 1 -> 写 0 -> 等到读回 0，指针归 0），
    // 这样下一个 SOCA 一定从 SOC0 开始，丢样和 SOC1~5 的对应关系才稳。
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 1;
    while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 1) {}
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0;
    while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 0) {}

    AdcRegs.ADCSOCFLG1.all    = 0xFFFF;     // 清掉全部 SOC 挂起标志
    AdcRegs.ADCINTFLGCLR.all  = 0x0001;     // 清 ADCINT1 旧标志
    AdcRegs.ADCINTOVFCLR.all  = 0x0001;     // 清 ADCINT1 溢出标志

    EDIS;
}

//===========================================================================
// GPIO 配置函数
//   GPIO5 中断计时脚，普通 GPIO 输出，初值低
//   spwm.c 只占用 GPIO0~GPIO3，所以 GPIO5 可以自由使用
//   （原来用 GPIO4，实测那个脚看不到脉冲，换到 GPIO5）
//===========================================================================
void Gpio_Config(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;         // 复用为普通 GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO5  = 1;         // 输出
    EDIS;

    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;        // 初值低
}

