/*
 * protect.h
 *
 * 纯软件保护：输入欠压 + 电感过流
 *
 * ★ 硬件保护已整体删除。原来的实现是 COMP1/COMP2 + 内部 DAC +
 *   ePWM Digital Compare + Trip Zone 中断，需要比较器的模拟通路，
 *   本板没有这条链路，配了也只会误动作。
 *
 * 现在保留的是两条软件判断，跑在 20kHz 的 ADC 中断里：
 *   欠压 UV : Vin < UV_Volts          -> 不锁存，切固定 0.85 正弦兜底（在 main.c 里做）
 *   过流 OC : |IL| > OC_Amps          -> 锁存，软件封波
 *
 * 封波仍然走 Trip Zone，但只用它的"软件强制"功能：
 *   TZCTL.TZA/TZB = force-low，TZSEL 全 0（没有任何外部触发源），
 *   只有 TZFRC.OST 能把四管拉低。
 * 走 TZ 而不是 AQCSFRC，是因为 AQCSFRC 只作用在 AQ 上，经死区模块取反后
 * EPWMxB 反而会变高（见 spwm.h 末尾的说明），不是真正的四管全关。
 *
 * Protect_Enable = 0（当前默认）时：两条判断照跑、标志照置，
 * 但不影响控制输出、也不封波，方便先观察真实工况。
 */

#ifndef PROTECT_H
#define PROTECT_H

#include "DSP28x_Project.h"
#include "IQmathLib.h"
#include "dcac_cfg.h"

// 欠压退出（回落）所需的连续正常拍数。UV_Debounce 那条在 dcac_cfg.h 常改区。
// 20 拍 @20kHz = 1ms，比 100Hz 母线的半周期(5ms)小得多，不会把真跌电压当成噪声
#define UV_Clear_Debounce   20U

// 故障码：main.c 根据下面两个标志算出来，OLED 用来显示
//   硬件 Trip 已经删掉了，只剩这两个
#define FAULT_NONE      (0U)
#define FAULT_UV        (1U)        // 输入欠压（不锁存）
#define FAULT_OC        (2U)        // 电感过流（锁存）

// 欠压标志：1 = Vin < UV_Volts（不锁存，电压回来自动清）
extern volatile Uint16 Protect_UvFault;
// 过流标志：1 = |IL| > OC_Amps（锁存，要 Protect_Clear() 才清）
extern volatile Uint16 Protect_OcFault;

// 去抖计数（调试观察用）
extern volatile Uint16 Protect_UvCnt;
extern volatile Uint16 Protect_UvClrCnt;
extern volatile Uint16 Protect_OcCnt;

// 无条件调用，即使 Protect_Enable==0 也要调：
// 它把 TZCTL 的 TZA/TZB 和四个 DCxEVT 通道都写成 force-low，并只选单次(OSHT)触发源。
// ★ 复位值 TZCTL = 0xFFFF，按位域拆开是 TZA=3/TZB=3/DCxEVTn=3，而 3 = "不动作"，
//   也就是复位状态下触发一次并不会关输出，必须显式写成 force-low 才有保护。
// ★ 不要为了"清干净"把 TZSEL 整个写 0：那样连 OSHT1..6 一起清掉，
//   单次触发源就没了，spec 要求 OSHT=1。
void Protect_Init(void);

// ADC 中断里每拍调用，参数是 Q24 的母线电压和电感电流
void Protect_Update(_iq vin, _iq il);

// 清过流锁存 + 清 TZ 触发，允许重新输出
void Protect_Clear(void);

// 软件封波：TZFRC.OST，四管全关（不产生 TZ 中断）
void Protect_ForceStop(void);

#endif
