/*
 * dcac_cfg.h
 *
 * 单相离网逆变硬件与控制配置（全 CPU 运行，不使用 CLA）
 *
 * 载波：ePWM1 增减计数 TBPRD=1500 -> fsw=20kHz
 * 采样：ePWM1 在 CTR=ZERO 触发 SOCA，ADCINT1 直接进 CPU 中断
 *   SOC0 丢弃
 *   SOC1 = ADCINA0 输出电流 i_o
 *   SOC2 = ADCINA1 输出电压 v_o
 *   SOC3 = ADCINA2 电感电流 i_L
 *   SOC4 = ADCB1 输入电压 v_in（分压 80/3.3，只监视/保护）
 *   SOC5 = ADCB0 输入电流 i_in（放大 150 倍，只监视/保护）
 *   控制环只吃 i_o / v_o / i_L，输入侧两路只做监视/保护（见 5.4 / 5.5）
 * 控制：电容电流 i_c = i_L - i_o
 *   电压外环 PR(准比例谐振) -> 电容电流给定 -> 电流内环 PI -> 调制比 mod
 *
 *
 * ============================ 定点化说明 ============================
 * 本工程控制环全部用 TI IQmath 定点运算，全局格式 GLOBAL_Q = 24（_iq）。
 *   Q24 分辨率 2^-24 = 5.96e-8，动态范围只有 ±128
 *   本工程所有运行量都在 ±128 以内：
 *     输出电压 80V、输出电流 22A、电感电流 30A、给定峰值 50.9V、
 *     电容电流给定 8A、调制比 0.85
 *   超过 ±128 的量必须换低 Q 存放，整个工程只有下面 3 处，都加了注释：
 *     a) 谐振角频率 w0 = 314  -> 设计期就折成 R_V = w0*Ts/2 = 0.00785，不再有 w0
 *     b) TBPRD/2 = 750        -> SPWM 里按 Q15 算比较值
 *     c) 2*满量程 = 160(电压) -> ADC 换算系数按 Q17 存放
 *
 * 两条必须记住的规矩：
 *   1) 中断里的乘法一律用 _IQmpy（它带 64 位中间结果并右移 GLOBAL_Q），
 *      直接写 a*b 得到的是 Q48 的截断值，是最隐蔽的错误来源；
 *   2) 加减法可以直接写，定点加减就是整数加减，不会溢出（除非超出 ±128）。
 * ====================================================================
 */

#ifndef CFG_H
#define CFG_H

// ---- 0. IQ 全局格式 ----
// IQmathLib.h 里 GLOBAL_Q 的默认值本来就是 24，这里显式写一遍：
// 换库版本或者有人在编译选项里 -dGLOBAL_Q=xx 时，不会悄悄变掉
#ifndef GLOBAL_Q
#define GLOBAL_Q 24
#endif
#include "IQmathLib.h"

#include "DSP28x_Project.h"

//===========================================================================
//                         ★★★ 常 改 区 ★★★
//  平时要动的基本只有这一段。下面的内容是标定、中断、RMS 之类的实现细节，
//  改之前先翻到对应小节看注释。
//===========================================================================

// ---- 环路模式 ----
// 0 正弦开环 + 软启动 / 1 电压单环 / 2 电流单环 / 3 电压+电流双环 / 4 固定占空比开环
// 允许 build configuration 用 -dLOOP_MODE=n 覆盖（必须 #ifndef 包住，否则头文件赢）
#define LOOP_SINE_OPEN    0
#define LOOP_VOLT_CLOSED  1
#define LOOP_CURR_CLOSED  2
#define LOOP_DUAL         3
#define LOOP_FIXED_DUTY   4
#ifndef LOOP_MODE
#define LOOP_MODE         2
#endif

// ---- 功能开关 ----
// SFRA_ENABLE: 0 = OLED 版（OLED + 示波器缓冲）；1 = SFRA 扫频版（无 OLED）
//   ★ 两者抢 GPIO28/29 的 SCI 引脚，且 SFRA 库只有 Flash 里放得下
// DEBUG_VIEW_ENABLE: 1 = 额外编译 Debug_View 的镜像字段和示波器缓冲；
//   0 = 只留 OLED 要用的那几个有效值。中断里两条路都只有一次函数调用/空调用
#ifndef SFRA_ENABLE
#define SFRA_ENABLE       0
#endif
#ifndef DEBUG_VIEW_ENABLE
#define DEBUG_VIEW_ENABLE 0
#endif

// ---- 电流环 PI（LOOP_CURR_CLOSED）----
// ★ 量纲随 Task 5（前馈）一起改：现在 PI 输出直接当调制比，所以是 mod/A；
//   加上前馈之后 PI 输出变成"电压"，这一对要改成 9.6 V/A / 11.46（× Vbus 60V）。
// Ki = 0 就是纯 P：被控对象是 1/(sL)，纯 P 在 50Hz 处 Kp/(wL) 已经足够大
#define KP_I                _IQ(0.16f)    // [mod/A]  -> 前馈后改 [V/A]
#define KI_I                0.0f          // [1/s]，0 = 纯 P

// ---- 双环 PI（LOOP_DUAL）----
// 双环那条路的 PI 输出直接当调制比用，量纲还是 mod/A，所以要单独一套，
// 不能和上面的 V/A 混用（混了会把双环增益静默放大 60 倍）
#define KP_I3               _IQ(0.16f)    // [mod/A]
#define KI_I3               0.0f

// ---- 保护 ----
// 纯软件判断，硬件保护（COMP/DAC/Digital Compare）已经删掉
#define UV_Volts_F          55.0f         // 输入欠压阈值 V
#define OC_Amps_F           12.0f         // 电感过流阈值 A（瞬时峰值）
#define UV_Debounce         200U          // 欠压去抖拍数（200 拍 @20kHz = 10ms）
#define OC_Debounce         3U            // 过流去抖拍数
// 0 = 只判断、只置标志，不影响输出（当前）
// 1 = 欠压切固定 0.85 正弦兜底，过流封波锁存
#ifndef Protect_Enable
#define Protect_Enable      0
#endif

//===========================================================================
//                        常改区到此为止
//===========================================================================

// ---- 1. 环路模式 / 功能开关：定义已移到上面的常改区 ----

// ---- 上位机 SFRA GUI（SCI-A 通讯）开关 ----
//   0 = 不接上位机：不初始化 SCI-A、不建映射表、中断里不服务协议。
//       扫频改为在 CCS Expressions 里给 SfraTrig 写 1 触发；扫完 SfraDone=1，
//       直接读 FreqVect / Plant_MagVect / Plant_PhaseVect / OL_* 导走即可。
//       好处：不需要串口硬件，也不再受 GUI 耐心(~60s)限制，想扫多少点都行。
//   1 = 恢复原来的 SFRA GUI 串口协议（和以前一样用 GUI 扫频）。
#ifndef SFRA_GUI_ENABLE
#define SFRA_GUI_ENABLE   0
#endif

// 运行模式标志：
//   0 = 配 28035_RAM_lnk.cmd（纯 RAM 运行，现在用这个；不掉电就不用重下）
//   1 = 配 28035_FLASH_lnk.cmd（Flash 运行，main() 里要先把 ramfuncs 搬到 RAM）
#ifndef SFRA_FLASH_BUILD
#define SFRA_FLASH_BUILD  0
#endif

// ---- 2. 系统时钟 / 载波 / 采样率 ----
// ePWM 增减计数：fsw = TBCLK / (2 * TBPRD)
//   TBCLK = 60MHz, TBPRD = 1500 -> fsw = 20kHz
//   TBCLK = 60MHz, TBPRD = 3000 -> fsw = 10kHz
// 单极性倍频：两个桥臂共用一个三角载波，各用一个比较值
//   引脚按走线方便分配：GPIO0/1 = PWM1AB = 右桥臂（EPWM1），
//                       GPIO2/3 = PWM2AB = 左桥臂（EPWM2）
//   EPWM1.CMPA = TBPRD/2 * (1 - mod)   右桥臂，占空 dR = (1 - mod)/2
//   EPWM2.CMPA = TBPRD/2 * (1 + mod)   左桥臂，占空 dL = (1 + mod)/2
//   TBPRD=3000 时即 1500 +- mod*1500
//   uAB = (dL - dR)*Vdc = mod*Vdc，电平只有 +Vdc/0/-Vdc，
//   每个载波周期两个脉冲，等效开关频率 = 2*fsw
#define TBCLK_Hz        60000000L
#define TBPRD           1500U
#define CMPA_Mid        (TBPRD / 2U)    // 750
#define CMPA_Span       (TBPRD / 2U)    // 750

// 下面两个 float 常量只在"设计期"用（算采样周期、相位步进、PR 系数、Ki*Ts），
// 它们只出现在 _IQ() 的参数里，编译期就被折成整数常量了，
// 中断里一个浮点运算都不会剩下
#define Fsw_Hz          ((float)TBCLK_Hz / (2.0f * (float)TBPRD))
#define Ts_S            (1.0f / Fsw_Hz)

// ---- 3. 输出规格 / 正弦表 / 相位步进 ----
// 输入 60V，输出 36Vrms/50Hz
//   Vref_Peak = sqrt(2)*36 = 50.91V
//   Mod_Max   = 50.91 / 60 = 0.85
#define Vbus_Nominal    60.0f
#define Fout_Hz         50.0f
#define Vout_Rms        36.0f
#define Vref_Peak_F     (1.41421356f * Vout_Rms)    // 50.91V，设计期
#define Mod_Max_F       0.85f
// 开环目标调制比幅值 = Vref_Peak / Vbus = 0.85
#define Mod_Amp_Open_F  (Vref_Peak_F / Vbus_Nominal)

#define Vref_Peak       _IQ(Vref_Peak_F)        // Q24：50.91V -> 854156389
#define Mod_Max         _IQ(Mod_Max_F)          // Q24：0.85
#define Mod_Amp_Open    _IQ(Mod_Amp_Open_F)     // Q24：0.85

#define Sin_Table_Size  256U
// phase_step = 256 * Fout / Fsw = 256*50/20000 = 0.64
#define Phase_Step_F    ((float)Sin_Table_Size * Fout_Hz / Fsw_Hz)

// 相位累加器用 Uint32 定点累加，不用 _iq：Q24 装不下 0~256 这个范围
//   高 8 位 = 正弦表下标 0..255，低 24 位 = 下标的小数部分（Q24）
//   每周期加 0.64，累加满 256 与 0x00FFFFFF 相与即自动回绕，
//   和原来的"Phase >= 256 就减 256"完全等价，但不要判断、不要减法
#define Phase_Acc_Bits  24U
#define Phase_Step_Q24  (Uint32)_IQ(Phase_Step_F)       // 0.64 -> 10737418
#define Phase_Mask      0xFFFFFFFFUL

// ---- 4. ADC 通道分配 ----
// F2803x 的 ADC 有 A/B 两组各 8 个通道，SOCxCTL.CHSEL 的编号是连续的：
//   A 组 ADCINA0~A7 = 通道 0~7
//   B 组 ADCINB0~B7 = 通道 8~15
// 所以 ADCB0 = 8、ADCB1 = 9。
// ADCB0 / ADCB1 在 F2803x 上是纯模拟脚（AIOMUX1 里对应位是保留位），
// 复位后就是模拟输入，不需要像 ADCINA2/A4 那样写 AIOMUX 配置。
//
// SOC 顺序 = 转换顺序：全部 SOC 共用同一个触发源（EPWM1 SOCA）、同样的优先级，
// 所以每个载波周期都按 SOC0 -> SOC1 -> ... 固定顺序转换。
// 这种"固定槽位"写法下第 N 个 SOC 的结果一定落在 ADCRESULTn 里，
// 不依赖轮询指针当拍停在哪儿，比"通道和 ADCRESULT 编号错开"稳得多。
//
// ★ SOC0 和 SOC1 必须读同一个通道，这是 TI 给的 rev0 芯片勘误修法：
//   每个 SOC 序列的"第一次采样"结果不准，所以同一通道连采两次、
//   丢掉第一次只用第二次。这两次必须紧邻，中间不能插别的通道，
//   否则第二次采到的也不是刚建立的那个值，修法就失效了。
//   本板子 ADCA0 = 输出电压，所以 SOC0/SOC1 都放 A0，真值取 SOC1。
#define ADC_Ch_Dummy    0U      // SOC0 -> ADCA0 丢样（与 SOC1 同通道）
#define ADC_Ch_Vout     0U      // SOC1 -> ADCA0 输出电压   ← 取这一次的
#define ADC_Ch_Iout     1U      // SOC2 -> ADCA1 输出电流
#define ADC_Ch_IL       2U      // SOC3 -> ADCA2 电感电流
#define ADC_Ch_Vin      9U      // SOC4 -> ADCB1 输入电压（只监视）
#define ADC_Ch_Iin      8U      // SOC5 -> ADCB0 输入电流（只监视）

// 中断触发点 = 最后一个 SOC。以后再加通道，这里必须跟着改，
// 否则 ADCINT1 会在新通道转换完成之前就进 ISR，读到的还是上一拍的结果
#define ADC_EOC_For_INT1 5U     // EOC5（SOC5 = i_in）触发 ADCINT1

// ============================================================================
// ---- 5. 采样前端标定 ----
//
// ★ 你只需要填两个数：Vzero 和 K，其余全部由编译器算好，不用手算码值。
//
// 每路调理电路送到 ADC 引脚的电压都是这个形式：
//
//     Vpin = Vzero + K * X
//
//   Vzero : 零输入时引脚上的电压，也就是偏置（V）。纯分压电路填 0
//   K     : 传递系数，单位 V/物理单位。
//           分压电路：K = 分压比 = 3.3 / 满量程
//           电流电路：K = Rsense * 放大倍数
//   X     : 要测的物理量（V 或 A）
//
// 下面这些中间量都是导出的，不要手改：
//   ADC_*_Offset = Vzero/Vref*4096   零点对应的 ADC 码（就近取整）
//   ADC_*_Span   = Vref/K            满码 4096 对应的物理量
//   ADC_*_PerCode = Span/4096        1 个码对应多少物理量（看分辨率用）
// ============================================================================

// ADC 满量程（参考）电压。
#define ADC_Vref            3.3f

// ---- 5.1 输出电流 i_o（ADCA1）----
// 1mΩ 采样电阻 + 20 倍增益：Vpin = Vzero + 0.02*I
// ★ Vzero 用实测值 2.409V，不是设计标称 2.5V：
//   本板 INA240 的参考电压实际就落在 2.409V（差 91mV = 4.5A）。
//   校零会再修正一次；但校零被一致性检查挡掉时，靠这里的正确标称值兜底
#define ADC_Iout_Vzero      2.409f      // 零电流时引脚电压 V（实测）
#define ADC_Iout_K          0.0177f       // V/A = 0.001 * 20

// ---- 5.2 输出电压 v_o（ADCA0）----
// 偏置 1.5V + 分压。Vpin = 1.5 + K*V，改 K 就改了量程
#define ADC_Vout_Vzero      1.5f
#define ADC_Vout_K          0.026f    //

// ---- 5.3 电感电流 i_L（ADCA2）----
// 和输出电流调理一样：1mΩ + 20 倍
// ★ 同样用实测值 2.431V（标称 2.5V 差 69mV = 3.5A）
#define ADC_IL_Vzero        2.431f
#define ADC_IL_K            0.0177f

// ---- 5.4 输入电压 v_in（ADCB1，直流）----
// 电阻分压，无偏置：Vpin = K*Vin
//   K = 3.3/80 -> 输入 80V 时引脚正好 3.3V
#define ADC_Vin_Vzero       0.0f
#define ADC_Vin_K           (3.3f / 80.0f)

// ---- 5.5 输入电流 i_in（ADCB0，直流）----
//   Vpin = K * Iin，K = Rsense * 放大倍数
#define ADC_Iin_Rsense      0.001f      // 采样电阻 Ω
#define ADC_Iin_Gain        150.0f      // 运放放大倍数
#define ADC_Iin_Vzero       0.0f        // 零电流引脚电压：运放无偏置，就是 0V
#define ADC_Iin_K           (ADC_Iin_Rsense * ADC_Iin_Gain)

// 零点码：Vzero 折算成 12 位码
#define ADC_OffsetCode(vzero)   ((long)((vzero) / ADC_Vref * 4096.0f + 0.5f))
// 全码跨度：整个 0~4096 码对应多少物理量（= ADC_Vref/K）
//   ★ 注意这个数**没有扣掉直流偏置**，不是实际能测的范围，别拿它当量程用,只用来算每码分辨率。
#define ADC_SpanOf(k)           (ADC_Vref / (k))
// 单个码的分辨率：Span/4096
#define ADC_PerCode(k)          (ADC_SpanOf(k) / 4096.0f)

// 实际可测范围：以零点码为中心，往下到 0 码、往上到满码（4096）
//   有偏置的通道上下不对称，这才是真正的量程
//   例：Vout Vzero=1.5V、K=0.027 -> 零点码 1862
//       往下 1862 码 = 55.56V，往上 (4096-1862) 码 = 66.67V
//       可测 -55.56V ~ +66.66V（不是 -61.1 ~ +61.1，也不是全跨度 122.22V）
//   无偏置的通道（Vzero=0）零点码为 0，范围就是 0 ~ SpanOf(K)
#define ADC_MinOf(vzero, k)     (-((float)ADC_OffsetCode(vzero) / 4096.0f * ADC_SpanOf(k)))
#define ADC_MaxOf(vzero, k)     ( (float)(4096L - ADC_OffsetCode(vzero)) / 4096.0f * ADC_SpanOf(k))

// 电流（IL / Iout 共用一套参数）
#define ADC_IL_Offset       ADC_OffsetCode(ADC_IL_Vzero)
#define ADC_IL_Span         ADC_SpanOf(ADC_IL_K)        // 全跨度，不是量程
#define ADC_IL_PerCode      ADC_PerCode(ADC_IL_K)
#define ADC_IL_Min          ADC_MinOf(ADC_IL_Vzero, ADC_IL_K)   // 真实可测下限
#define ADC_IL_Max          ADC_MaxOf(ADC_IL_Vzero, ADC_IL_K)   // 真实可测上限
#define ADC_Iout_Offset     ADC_OffsetCode(ADC_Iout_Vzero)
#define ADC_Iout_Span       ADC_SpanOf(ADC_Iout_K)
#define ADC_Iout_PerCode    ADC_PerCode(ADC_Iout_K)
#define ADC_Iout_Min        ADC_MinOf(ADC_Iout_Vzero, ADC_Iout_K)
#define ADC_Iout_Max        ADC_MaxOf(ADC_Iout_Vzero, ADC_Iout_K)

// 输出电压
#define ADC_Vout_Offset     ADC_OffsetCode(ADC_Vout_Vzero)
#define ADC_Vout_Span       ADC_SpanOf(ADC_Vout_K)
#define ADC_Vout_PerCode    ADC_PerCode(ADC_Vout_K)
#define ADC_Vout_Min        ADC_MinOf(ADC_Vout_Vzero, ADC_Vout_K)
#define ADC_Vout_Max        ADC_MaxOf(ADC_Vout_Vzero, ADC_Vout_K)

// 输入电压（直流，无偏置）
#define ADC_Vin_Offset      ADC_OffsetCode(ADC_Vin_Vzero)
#define ADC_Vin_Span        ADC_SpanOf(ADC_Vin_K)
#define ADC_Vin_PerCode     ADC_PerCode(ADC_Vin_K)
#define ADC_Vin_Min         ADC_MinOf(ADC_Vin_Vzero, ADC_Vin_K)
#define ADC_Vin_Max         ADC_MaxOf(ADC_Vin_Vzero, ADC_Vin_K)

// 输入电流（直流，运放无偏置）
#define ADC_Iin_Offset      ADC_OffsetCode(ADC_Iin_Vzero)
#define ADC_Iin_Span        ADC_SpanOf(ADC_Iin_K)
#define ADC_Iin_PerCode     ADC_PerCode(ADC_Iin_K)
#define ADC_Iin_Min         ADC_MinOf(ADC_Iin_Vzero, ADC_Iin_K)
#define ADC_Iin_Max         ADC_MaxOf(ADC_Iin_Vzero, ADC_Iin_K)

// ---- 当前参数下编译器会算出这些值（对照用，改完 K/Vzero 后自己核一下）----
//   通道     Vzero     K     零点码      全跨度                  可测范围                  1码分辨率
//   Vout  1.5V    0.027   1862    122.22V   -55.56V ~ +66.66V    29.84mV
//   Iout  2.5V    0.02    3103    165.00A   -125.0A ~ +40.00A    40.28mA
//   IL    2.5V    0.02    3103    165.00A   -125.0A ~ +40.00A    40.28mA
//   Vin   0V      3.3/80  0       80.00V      0.00V ~ +80.00V    19.53mV
//   Iin   0V      0.15    0       22.00A      0.00A ~ +22.00A     5.37mA
//
//   对比：Vout 原来 K=0.03125 时
//         全跨度 105.60V，可测 -48.00V ~ +57.60V，分辨率 25.78mV
//   K 变小 -> 全跨度和可测范围都变大，但分辨率变粗
//
//   无偏置的两路（Vin/Iin）零点码为 0，范围就是 0 ~ 全跨度，不存在不对称

// ---- 5b. ADC 零点校准（软件校零） ----
// 上面那组 Offset 是"标称值"，由各路的 Vzero 偏置算出来（Vout 1.5V、
// Iout/IL 2.5V、Vin/Iin 无偏置为 0）。实际板子元件有误差和温漂，
// 真零点会偏几十个码。校零做的事：让各路调理电路处于静止零输入，
// 把这几十/几百拍的 ADC 码值平均出来，当成真正的零点码。
// 启用条件：校零的那一下，各路输入必须真的是零
// 满足不了就把 ADC_ZeroCal_Enable 置 0，用上面的标称 Offset 也能跑，
// 只是读数会带上板子的实际零点误差
#define ADC_ZeroCal_Enable  1

// 每通道平均多少个采样点
// 1024 点 @20kHz = 51.2ms，噪声按 sqrt(N) 再降一半（相对 256 点）
#define ADC_ZeroCal_Samples 1024U
#define ADC_ZeroCal_Shift   10U     // log2(ADC_ZeroCal_Samples)

// 一致性检查：用前 256 点的 (最大-最小) 判断这一路是不是悬空/在跳。
// 超过这个码数就认为该通道不可信，放弃校准、保留标称 Offset。
// 静止时正常应该在几个码以内。
// ★ 如果误报了，先确认这一路是不是真的悬空；确实需要放宽再调大这个值。
//   放宽会连带接受"校零时已经有电流/电压"的情况，别调太猛
#define ADC_ZeroCal_MaxSpread   64
#define ADC_ZeroCal_SpreadN     256U    // 用前多少点做一致性检查

// 定点换算 value = (raw - Offset) * Span / 4096：
//   (raw-Offset) << 12   = (raw-Offset)/4096 的 Q24，最大 3103*4096 = 1.3e7，不溢出
//   Span 乘 2^17 做成 Q17 常量：165 * 131072 = 2.16e7，32 位装得下
//   __IQmpy(A,B,Q) = (A*B)>>Q，Q24 × Q17 >> 17 的结果正好还是 Q24
// Span 本身超过 Q24 的 ±128（165），所以只能走 Q17，不能写成 _IQmpy
// 注：浮点乘出来再截断成整数，可能比精确值小 1（例：22.0*131072 = 2883583.9999
//     -> 2883583）。相对误差 ~3e-7，比 1 个 ADC 码（1/4096 = 2.4e-4）小三个
//     数量级，对换算精度没有影响
#define ADC_Scale_Q17(span) ((long)((span) * 131072.0f))
#define ADC_To_IQ(raw, off, span) \
    __IQmpy((_iq)(((long)(raw) - (long)(off)) << 12), ADC_Scale_Q17(span), 17)

// 校零之后的换算：零点码不再是编译期常量，而是 main.c 里实测出来的 OffsetNeg
//   记标称零点码 nominal（编译期常量，就是上面的 ADC_*_Offset），
//      实测零点码 raw_zero（运行期测出来的），
//      offneg = nominal - raw_zero
//   要算的是 (raw - raw_zero)，而
//     raw - raw_zero = (raw - nominal) + (nominal - raw_zero)
//                    = (raw - nominal) + offneg
//   所以：
//     (raw - raw_zero)/4096 的 Q24 = ((raw - nominal) + offneg) << 12
//   再乘 Span：和 ADC_To_IQ 完全同构，只是左操作数从编译期常量
//   变成了运行期算术，右操作数仍然是编译期常量 Span*2^17
//
// 量级检查：|raw - raw_zero| <= 4095，<<12 后 <= 1.68e7，做 _IQ() 不会溢出；
// 乘 Span*2^17 时 __IQmpy 走 32x32->64，也不会溢出
#define ADC_To_IQ_Cal(raw, nominal, offneg, span) \
    __IQmpy((_iq)((((long)(raw) - (long)(nominal)) + (long)(offneg)) << 12), \
            ADC_Scale_Q17(span), 17)

// 输入侧两路（直流）的换算和上面同构：
//   vin = ((raw - ADC_Vin_Offset) + VinOffNeg) * ADC_Vin_Span / 4096
//   iin = ((raw - ADC_Iin_Offset) + IinOffNeg) * ADC_Iin_Span / 4096
// 后面加保护阈值时，既可以直接比码值（省一次乘法），也可以按上面公式算成
// Q24 再和 _IQ(阈值) 比

// ---- 6. 控制环参数（待整定） ----
#define W0_V_F          314.159265f     // 电压环 2*pi*50 谐振角频率
#define Wc_V_F          18.0f           // 电压环准 PR 截止角频率
// PR 的 Tustin 系数只需要下面两个无量纲小量，设计期一次算好：
//   R_V = w0*Ts/2 = 314.159*50us/2 = 7.853982e-3  Q24 -> 131767
//   Q_V = wc*Ts/2 = 10*50us/2      = 2.5e-4       Q24 -> 4194
// w0 = 314 本身超出 Q24 的 ±128，做成 Q8 再乘会有精度损失，
// 直接用编译期常量 _IQ(公式) 算，结果是 Q24 能表示的最接近值
#define R_V             _IQ(W0_V_F * Ts_S * 0.5f)
#define Q_V             _IQ(Wc_V_F * Ts_S * 0.5f)

#define KP_V            _IQ(0.0518f)    // 电压环比例
#define KR_V            _IQ(0.0823f)    // 电压环谐振增益

// KP_I / KI_I / KP_I3 / KI_I3 的定义在顶部常改区。
// 注意量纲：KP_I/KI_I 是 mode 2（前馈结构）用的 V/A，KP_I3/KI_I3 是 mode 3 用的 mod/A。
// 积分增益把采样周期折进去：KI_I_Ts = KI_I*Ts，中断里只乘一次。
// 这样做一是快，二是保精度：Ts = 50us 在 Q24 里只有 838 个刻度
// （0.1% 量化误差），先乘 Ts 会把误差的有效位数吃掉十几位。
// 折进来以后 KI_I_Ts = KI_I*50us，量级很小，Q24 的 6e-8 分辨率完全够用
#define KI_I_Ts         _IQ(KI_I  * Ts_S)
#define KI_I3_Ts        _IQ(KI_I3 * Ts_S)

#define Iref_Max_F      8.0f            // 电容电流给定限幅 A
#define Iref_Max        _IQ(Iref_Max_F)

// ---- 7. 保护总开关 ----
// Protect_Enable 的定义在顶部常改区。
// 硬件保护（COMP1/COMP2 + 内部 DAC + Digital Compare + TZ 中断）已整体删除，
// 板上没有比较器通路。现在只有 protect.c 里的两条软件判断：
//   欠压 Vin < UV_Volts、过流 |IL| > OC_Amps

// ---- 8. 保护阈值（Q24 化）与软启动 ----
// UV_Volts_F / OC_Amps_F / UV_Debounce / OC_Debounce 在顶部常改区
#define UV_Volts        _IQ(UV_Volts_F)
#define OC_Amps         _IQ(OC_Amps_F)
#define Soft_Start_Ms   500U            // 软启动时长 ms
// 每个中断周期给定增量 = 额定峰值 / (软启动时间 * 中断频率)
#define Soft_Start_Step_F \
    (Vref_Peak_F / ((float)Soft_Start_Ms * 0.001f * Fsw_Hz))
#define Soft_Start_Step _IQ(Soft_Start_Step_F)      // Q24：5.0912e-3 -> 85415

// ---- 8b. 电流单环（LOOP_CURR_CLOSED）给定 ----
// ★ 单电流环没有电压环兜底，给定必须是"安培"，不能再用电压环的 Vref：
//   Vref = sin*RefRamp 的幅值会爬到 Vref_Peak = 50.91，
//   直接当电流给定就是 50.9A，PI 一拍就顶到 ±Mod_Max，
//   四管打满、电流失控（Protect_Enable=0 时没有任何保护）。
// ★ 所以电流给定有自己的软启动斜坡 IRamp，从 0 爬到 Iref_Peak。
//
// 给定形式：50Hz 正弦，有效值 4.16A
//   峰值 = sqrt(2)*4.16 = 5.883A，Q24 的动态范围 ±128 装得下
#define Iref_Rms_F          4.16f
#define Iref_Peak_F         (1.41421356f * Iref_Rms_F)   // 5.883A 峰值
#define Iref_Peak           _IQ(Iref_Peak_F)
// 每个中断周期给定幅值的增量：和电压环共用 Soft_Start_Ms 这个软启动时长
#define Iref_Ref_Step_F \
    (Iref_Peak_F / ((float)Soft_Start_Ms * 0.001f * Fsw_Hz))
#define Iref_Ref_Step       _IQ(Iref_Ref_Step_F)        // Q24：5.883e-4 -> 9870

// ---- 9. 中断计时脚 ----
// GPIO4 配成普通 GPIO 输出，进 ADC 中断拉高、退出前拉低
// 用它判断两件事：有没有脉冲 = ADC 中断在不在跑；脉宽 = 控制环耗时
// 定点化以后控制环应该明显快于原来的软件浮点版本，可以直接用脉宽对比
#define ISR_Test_Pin    4U

// ---- 10. 调试器观察用浮点镜像 ----
// 控制量都是 Q24，调试器里显示的是原始整数（真值 = 原始整数 / 16777216），
// 看着不直观。置 1 时在后台死循环里把它们换算成工程量存进 main.c 的
// Debug_View，调试器里把 Debug_View 加进 Expressions 展开就是 V / A / 调制比。
// 换算只在后台空转的循环里做，20kHz 中断里一次都不做，不影响控制时序；
// 置 0 时 Debug_View 和换算代码都不编译，不占 Flash 也不多跑一条指令
#define Debug_Float_View    1
// 后台循环每转多少圈折算一次：调小刷新快、后台占用高
#define Debug_Div           256U

// ---- 11. 有效值 / 平均值计算（只看，不进控制环） ----
// 做法参考 TI 参考设计 voltagesourceinvlcfltr 里的 sineAnalyzerWithPowerMeas.c：
//   ISR 里每拍累加平方和 sum(x2)，满一个工频周期后 RMS = sqrt(sum(x2)/N)。
//   这是有效值的定义式，对任意波形都精确；噪声进的是功率平均，
//   不会被当成峰值放大，所以比"峰值保持"那套又短又准。
//
// 和 TI 原版的差别（为什么不原样照抄）：
//   TI 用"过零检测"判周期（prevSign/currSign），再配 nSamplesMin/Max
//   做电网频率健全性判断。本工程输出是固定 50Hz、中断固定 20kHz，
//   两者严格成整数比，每 Rms_Cycle_Samples 拍正好一个整周期，
//   直接用计数更简单也更稳，不需要过零逻辑。
//   TI 那个 threshold = 0.08 是它 620V 量程下的归一化死区（约 50V），
//   本工程 24V 量级根本不能用，所以这里不做阈值判断。
//
// 累加器用 64 位（long）：
//   sum(x2) 是 Q48 量级。以电容电流为例，|x| 最大 8A = 8*2^24 ≈ 1.34e8，
//   x2 ≈ 1.8e16，乘 400 拍 ≈ 7.2e18，64 位上限 9.2e18，装得下。
//   量程 128 那种极端值不在本工程出现（见 dcac_cfg.h 开头的定点化说明）。
//
// 取 400 拍 = Fsw/Fout = 20000/50，正好一个工频周期
#define Rms_Cycle_Samples   400U

#endif
