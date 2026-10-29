/*
 * debug_view.c
 *
 * 调试镜像 + 有效值 + 示波器缓冲的具体实现，接口见 debug_view.h。
 *
 * 数据流：
 *   20kHz ISR   -> Debug_View_IsrUpdate()   平方累加、取 |Mod| 峰值（纯定点整数）
 *               -> Debug_View_ScopeSample() 按 Scope_Prescale 抽点存 float
 *   后台空转    -> Debug_View_Background()  每满一个工频周期开方一次、IIR 平滑
 *
 * ★ ISR 里没有开方、没有除法、没有浮点。开方和 float 除法全在后台。
 */

#include "debug_view.h"
#include "dcac_cfg.h"
#include <math.h>

//===========================================================================
// 有效值 / 平均值累加器（真有效值，做法参考 TI sineAnalyzerWithPowerMeas）
// C28x 的 long 是 32 位，累加平方会溢出，所以累加器必须用 long long；
// 而且 x 先右移 8 位（Q24->Q16）再平方，否则 400 拍同样会溢出：
//   51V 时 (51*2^16)^2 * 400 = 4.5e15 < 9.2e18  OK
//===========================================================================
static volatile long long RmsVoutSq = 0;
static volatile long long RmsIoutSq = 0;
static volatile long long RmsILSq   = 0;
static volatile long long AvgVinSum = 0;
static volatile long long AvgIinSum = 0;

// 调制比用一个工频周期内的峰值保持，不平方：
//   Mod 是正弦，幅值就是它的峰值，直接取 |Mod| 的最大值最直观，
//   而且削顶时读到的正好是限幅值（比如 0.85），不会像 RMS*sqrt(2)
//   那样把方波算成 0.85*1.414=1.2 这种虚高值
static volatile _iq ModPk = 0;

// RMS 平滑滤波状态（Q24 定点，每 20ms 更新一次）
static volatile _iq FiltVout = 0;
static volatile _iq FiltIout = 0;
static volatile _iq FiltIL   = 0;
static volatile _iq FiltMod  = 0;

// 校零实测到的引脚电压（V），只给 OLED 显示 Z: 用
static float CalPinVout = 0.0f;
static float CalPinIout = 0.0f;
static float CalPinIL   = 0.0f;

// 滑窗口长度：不在这里定义，共用 dcac_cfg.h 的 Rms_Cycle_Samples
// 一阶 IIR 平滑系数 1/2^Rms_SmoothShift（更新率 50Hz，不是 20kHz）
#define Rms_SmoothShift 2U

//===========================================================================
// 示波器数据缓冲区：CCS 的 Graph 工具按"数组名 + 长度"读一段连续内存，
// 所以必须用全局数组。存的是瞬时值换算成工程量的 float，
// Graph 里 Data Type 选 "32 bit floating point"、Q_Value=0 即可。
//
// 只有 OLED 版（SFRA_ENABLE==0）且 DEBUG_VIEW_ENABLE==1 才需要；
// 默认关掉，省 512 字数据 RAM（RAM 版已经很紧）。
//===========================================================================
#define Scope_Size      128U

#if (SFRA_ENABLE == 0) && (DEBUG_VIEW_ENABLE == 1)
#pragma DATA_SECTION(ScopeIout, "scopeBufs")
#pragma DATA_SECTION(ScopeIL,   "scopeBufs")

volatile float ScopeIout[Scope_Size];   // 输出电流瞬时值 A
volatile float ScopeIL[Scope_Size];     // 电感电流瞬时值 A
volatile Uint16 ScopeIdx = 0U;          // 写指针
volatile Uint16 ScopeCnt = 0U;          // 抽取计数

// 抽取倍数：每 N 个中断存 1 点（128 点一屏）。
//   1 -> 20kHz，一屏 6.4ms  = 0.3 个工频周期（看开关纹波细节）
//   3 -> 6.7kHz，一屏 19ms  = 1 个工频周期
//   5 ->  4kHz，一屏 32ms  = 1.6 个工频周期
// 改完 Graph 的 Sampling Rate Hz 要跟着改成 20000/N
volatile Uint16 Scope_Prescale = 5U;
#else
// 关掉时给一组空壳，保证 .graphProp 里的符号名仍然存在（读到的全是 0）
volatile float  ScopeIout[1];
volatile float  ScopeIL[1];
volatile Uint16 ScopeIdx = 0U;
volatile Uint16 Scope_Prescale = 5U;
#endif

//===========================================================================
// 有效值镜像结构体：OLED 全部数据来源 + Expressions 观察用
//===========================================================================
volatile Debug_View_Type Debug_View;

/*===========================================================================
 * 函数名：Debug_View_Init
 *===========================================================================*/
void Debug_View_Init(void)
{
    RmsVoutSq = 0;
    RmsIoutSq = 0;
    RmsILSq   = 0;
    AvgVinSum = 0;
    AvgIinSum = 0;
    ModPk     = 0;

    FiltVout = 0;
    FiltIout = 0;
    FiltIL   = 0;
    FiltMod  = 0;

    Debug_View.VoutRms = 0.0f;
    Debug_View.IoutRms = 0.0f;
    Debug_View.VoutAmp = 0.0f;
    Debug_View.IoutAmp = 0.0f;
    Debug_View.ILRms   = 0.0f;
    Debug_View.ModAmp  = 0.0f;
    Debug_View.Vin     = 0.0f;
    Debug_View.Iin     = 0.0f;
}

/*===========================================================================
 * 函数名：Debug_View_SetCalPin
 * 说明：Adc_ZeroCal 里调一次，把实测引脚电压交给 OLED 的 Z: 显示
 *===========================================================================*/
void Debug_View_SetCalPin(float vo, float io, float il)
{
    CalPinVout = vo;
    CalPinIout = io;
    CalPinIL   = il;
}

/*===========================================================================
 * 函数名：Debug_View_IsrUpdate
 * 说明：20kHz 中断里每拍调用。只做平方累加和取峰值，开方留到后台。
 *       先右移 8 位（Q24 -> Q16）再平方，否则 400 拍会溢出。
 *===========================================================================*/
void Debug_View_IsrUpdate(_iq vout, _iq iout, _iq il, _iq vin, _iq iin, _iq mod)
{
    long xv = (long)(vout >> 8);
    long xi = (long)(iout >> 8);
    long xl = (long)(il   >> 8);

    RmsVoutSq += (long long)xv * (long long)xv;
    RmsIoutSq += (long long)xi * (long long)xi;
    RmsILSq   += (long long)xl * (long long)xl;

    AvgVinSum += (long long)vin;
    AvgIinSum += (long long)iin;

    // 调制比取峰值：|Mod| 越大就越新，一个周期里留最大值
    {
        _iq ax = (mod >= 0) ? mod : -mod;

        if (ax > ModPk)
        {
            ModPk = ax;
        }
    }
}

/*===========================================================================
 * 函数名：Debug_View_ScopeSample
 * 说明：示波器缓冲抽样。内部自带抽取计数，Scope_Prescale 防 0。
 *       功能没打开时是空函数，编译器直接内联掉。
 *===========================================================================*/
void Debug_View_ScopeSample(_iq iout, _iq il)
{
#if (SFRA_ENABLE == 0) && (DEBUG_VIEW_ENABLE == 1)
    Uint16 pre = Scope_Prescale;

    if (pre == 0U)
    {
        pre = 1U;               // 防止在 Expressions 里误改成 0
    }

    if (++ScopeCnt >= pre)
    {
        ScopeCnt = 0U;

        ScopeIout[ScopeIdx] = _IQtoF(iout);
        ScopeIL[ScopeIdx]   = _IQtoF(il);

        ScopeIdx++;
        if (ScopeIdx >= Scope_Size)
        {
            ScopeIdx = 0U;
        }
    }
#else
    (void)iout;
    (void)il;
#endif
}

/*===========================================================================
 * 函数名：Debug_View_Background
 * 说明：后台调用（每 20ms 一次，由 main.c 的 RmsCommit 节拍触发）
 *       用 ISR 累加好的平方和算有效值 -> 开方 -> IIR 平滑 -> 刷新镜像
 *===========================================================================*/
void Debug_View_Background(void)
{
#if (SFRA_ENABLE == 0)
    float msV;
    float msI;
    float msL;

    // 累加器是 sum((x>>8)^2) = sum(A^2 * 2^32)
    // 均值 = /N，再 /2^32 就回到 A^2 的真实值（V^2 / A^2）
    msV = (float)RmsVoutSq / (float)Rms_Cycle_Samples / 4294967296.0f;
    msI = (float)RmsIoutSq / (float)Rms_Cycle_Samples / 4294967296.0f;
    msL = (float)RmsILSq   / (float)Rms_Cycle_Samples / 4294967296.0f;

    // 开方得真实有效值，再走一阶 IIR 平滑：y += (x - y) / 2^Rms_SmoothShift
    {
        _iq xv = (_iq)(sqrtf(msV) * 16777216.0f);
        _iq xi = (_iq)(sqrtf(msI) * 16777216.0f);
        _iq xl = (_iq)(sqrtf(msL) * 16777216.0f);
        _iq xm = ModPk;         // 本周期 |Mod| 的峰值

        ModPk = 0;              // 清掉，下个周期重新找

        FiltVout += (xv - FiltVout) >> Rms_SmoothShift;
        FiltIout += (xi - FiltIout) >> Rms_SmoothShift;
        FiltIL   += (xl - FiltIL)   >> Rms_SmoothShift;
        FiltMod  += (xm - FiltMod)  >> Rms_SmoothShift;

        Debug_View.VoutRms = _IQtoF(FiltVout);
        Debug_View.IoutRms = _IQtoF(FiltIout);
        Debug_View.ILRms   = _IQtoF(FiltIL);
        Debug_View.ModAmp  = _IQtoF(FiltMod);   // 峰值就是幅值，不用再乘
    }

    Debug_View.VoutAmp = Debug_View.VoutRms * 1.41421356f;
    Debug_View.IoutAmp = Debug_View.IoutRms * 1.41421356f;

    Debug_View.Vin = (float)AvgVinSum / (float)Rms_Cycle_Samples / 16777216.0f;
    Debug_View.Iin = (float)AvgIinSum / (float)Rms_Cycle_Samples / 16777216.0f;

    RmsVoutSq = 0;
    RmsIoutSq = 0;
    RmsILSq   = 0;
    AvgVinSum = 0;
    AvgIinSum = 0;
#endif

    // 校零实测的引脚电压（Adc_ZeroCal 里只算一次，这里搬到 Debug_View 给 OLED 用）
    Debug_View.CalPinVout = CalPinVout;
    Debug_View.CalPinIout = CalPinIout;
    Debug_View.CalPinIL   = CalPinIL;
}
