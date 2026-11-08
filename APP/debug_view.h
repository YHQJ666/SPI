/*
 * debug_view.h
 *
 * 调试镜像：把 Q24 的控制量换算成工程量的 float，给 CCS Expressions 和
 * OLED 共用；外加真有效值/平均值累加和示波器缓冲。
 *
 * 为什么单独一个文件：这些代码只在"看"的时候用，20kHz 控制环一条都不用，
 * 堆在 main.c 开头会把真正要改的开关淹掉。
 *
 * 分两层编译（见 dcac_cfg.h 的 DEBUG_VIEW_ENABLE）：
 *   OLED 版（SFRA_ENABLE==0）：有效值 + Debug_View 的 6 个显示字段一定要算，
 *                              因为 OLED 全部数据都来自这里
 *   DEBUG_VIEW_ENABLE==1      ：再额外打开示波器缓冲和 VoutAmp/IoutAmp/Iin
 *
 * ★ 注意：ISR 里只做"累加"，不做开方和除法。开方/floating 全在后台做。
 */

#ifndef DEBUG_VIEW_H
#define DEBUG_VIEW_H

#include "DSP28x_Project.h"
#include "IQmathLib.h"
#include "dcac_cfg.h"

// 调试器里加这个结构体展开看，全是工程量（V / A / 调制比）
// ★ 用 typedef 而不是匿名 struct：匿名 struct 在每个 .c 里都是不同的类型，
//   头文件里 extern 一个匿名 struct、.c 里再定义另一个，编译器会判类型不兼容
typedef struct
{
    // 有效值 / 幅值（滤波后）
    float VoutRms;      // 输出电压有效值 Vrms
    float IoutRms;      // 输出电流有效值 Arms
    float VoutAmp;      // 输出电压幅值 V（= VoutRms*1.414）
    float IoutAmp;      // 输出电流幅值 A
    float ILRms;        // 电感电流有效值 Arms
    float ModAmp;       // 调制比幅值（一个工频周期内的 |Mod| 峰值）

    // 输入侧（直流）
    float Vin;          // 输入电压 V
    float Iin;          // 输入电流 A

    // 校零时三个有偏置通道的引脚电压（V），OLED 的 Z: 用
    float CalPinIout;
    float CalPinVout;
    float CalPinIL;
} Debug_View_Type;

extern volatile Debug_View_Type Debug_View;

//===========================================================================
// 电流环健康度：用来区分"跟不上"是哪一类原因
//   饱和率高          -> 没电压余量（母线太低 / 负载太重 / 给定太大），工况问题
//   饱和率低但误差大  -> 环路增益或相位不对，调参问题
// 每 20ms（一个工频周期）刷新一次
//===========================================================================
typedef struct
{
    float V_L_Amp;      // |V_L| 峰值，V —— 峰值保持，单个毛刺就会顶满，只适合查尖峰
    float IErr_Amp;     // |IRef - IL| 峰值，A —— 同上，别拿它判断跟得准不准
    float IErr_Rms;     // (IRef - IL) 的有效值，A —— ★ 看跟踪精度用这个
    float ModSatPct;    // |Mod| >= Mod_Max 的采样点占比，% —— 调制比贴限幅的程度
    float PiSatPct;     // PI 输出被 OutMax/OutMin 限住的占比，% —— 余量耗尽的程度
    float VinFF;        // 滤波后的母线电压，V（InvVin 就是它的倒数）
    // ---- 滤波前后对照（查一阶 IIR 是否正常）----
    // ★ 判据是**幅度**，不是瞬时值：20ms 提交一次 = 正好一个 50Hz 周期，
    //   瞬时值永远落在同一个相位上，所以看着"固定不变"是必然的。
    //   VoutFF 的峰值应该 ≈ Vout 的峰值 = VoutRms*1.414（50Hz 上增益 0.999）
    float VoutFF_Amp;   // |VoutFF| 峰值，V   ← 应 ≈ VoutAmp
    float VinFF_Amp;    // |VinFF| 峰值，V    ← 应 ≈ 母线直流值
    // 下面三个只是瞬时快照（Expressions 里看，不上屏）
    float Vout;         // 输出电压瞬时值（未滤波），V
    float VoutFF;       // 输出电压前馈（滤波后）瞬时值，V
    float Vin;          // 母线瞬时值（未滤波），V
} Debug_Loop_Type;

extern volatile Debug_Loop_Type Debug_Loop;

// 示波器缓冲（只在 DEBUG_VIEW_ENABLE==1 且 SFRA_ENABLE==0 时有内容）
extern volatile float    ScopeIout[];
extern volatile float    ScopeIL[];
extern volatile Uint16   ScopeIdx;
extern volatile Uint16   Scope_Prescale;   // 抽取倍数，可在 Expressions 里改

// 初始化（清零累加器和滤波器状态）
void Debug_View_Init(void);

// ADC 中断里每拍调用：只做平方累加 / 取峰值，无开方无浮点
void Debug_View_IsrUpdate(_iq vout, _iq iout, _iq il, _iq vin, _iq iin, _iq mod);

// ADC 中断里每拍调用：示波器缓冲抽样（内部自带抽取计数和防 0）
void Debug_View_ScopeSample(_iq iout, _iq il);

// ADC 中断里每拍调用（**只在前馈更新之后**）：VoutFF / VinFF 的峰值保持
// 为什么单独一个函数：VoutFF 是在电流环分支里、Debug_View_IsrUpdate 之后才更新的，
// 想拿到"本周期的滤波结果"就必须在这里再取一次峰值
void Debug_View_IsrFf(_iq voutff, _iq vinff);

// 后台调用：每满一个工频周期算一次有效值、平滑、刷新 Debug_View
void Debug_View_Background(void);

// Adc_ZeroCal 里调一次，把实测的引脚电压交给显示用
void Debug_View_SetCalPin(float vo, float io, float il);

#endif
