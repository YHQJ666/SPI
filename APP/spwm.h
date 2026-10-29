/*
 * spwm.h
 *
 * 单相全桥单极性倍频 SPWM（一个三角载波 + 两路反相调制波）
 *
 * 方案：
 *   载波   : EPWM1 增减计数 TBPRD=1500，EPWM2 与 EPWM1 同步、同周期，
 *            两个桥臂共用这一路三角载波
 *   调制波 : 左桥臂 sinθ，右桥臂 -sinθ，共用一个 256 点正弦表
 *   比较值 : cmpA = TBPRD/2 * (1 - m*sinθ)   右桥臂（EPWM1，GPIO0/1）
 *            cmpB = TBPRD/2 * (1 + m*sinθ)   左桥臂（EPWM2，GPIO2/3）
 *            两者之和恒等于 TBPRD
 *   动作   : AQCTLA/AQCTLB 都写成 CAU=CLEAR, CAD=SET
 *            -> 占空 = CMPA/TBPRD，且 B 是 A 的互补
 *            死区子模块 IN_MODE=DBA_ALL / OUT_MODE=FULL / POLSEL=AHC，
 *            每个桥臂上下管互补并插入 300ns 死区
 *
 * 引脚与占空：
 *   GPIO0 = EPWM1A 右桥臂上管，占空 dR = cmpA/TBPRD = (1 - m*sinθ)/2
 *   GPIO1 = EPWM1B 右桥臂下管，与 GPIO0 互补（含死区）
 *   GPIO2 = EPWM2A 左桥臂上管，占空 dL = cmpB/TBPRD = (1 + m*sinθ)/2
 *   GPIO3 = EPWM2B 左桥臂下管，与 GPIO2 互补（含死区）
 *
 * 输出合成：
 *   uAB = uL - uR = (dL - dR)*Vdc = m*sinθ*Vdc
 *   电平只有 +Vdc / 0 / -Vdc 三种，uAB 每个载波周期出现两个脉冲，
 *   等效开关频率 = 2*fsw = 40kHz，滤波电感电容可以显著减小
 *
 *   m 上限 0.85 -> 基波峰值 0.85*60V = 51V = 36Vrms
 *
 * GPIO 占用：只占 GPIO0~GPIO3，GPIO4 留给中断计时脚
 *
 * 定点化：调制比 mod 是 Q24（±1），比较值幅度 750 超过 Q24 的 ±128，
 *         所以在 SPWM_SetMod 内部降到 Q15 再乘，见 spwm.c
 */

#ifndef SPWM_H_
#define SPWM_H_

#include "DSP28x_Project.h"
#include "IQmathLib.h"
#include "dcac_cfg.h"

// 死区 300ns @60MHz TBCLK -> 18 个计数
#define SPWM_DEADTIME_NS        (300UL)
#define SPWM_TBCLK_NS           (1000000000UL / 60000000UL)
#define SPWM_DB_COUNTS          ((Uint16)(SPWM_DEADTIME_NS / SPWM_TBCLK_NS))

// ePWM + GPIO 初始化，含 ADC 的 SOCA 触发源，并关闭 PWM 中断
void SPWM_Init(void);

// 写调制比（Q24，-1 ~ +1），内部按单极性倍频算出左右桥臂两个比较值
//   EPWM1.CMPA = TBPRD/2*(1 - mod)（右桥臂），EPWM2.CMPA = TBPRD/2*(1 + mod)（左桥臂）
void SPWM_SetMod(_iq mod);

// 输出中点，mod=0，两桥臂都 50%，交流侧平均 0V
void SPWM_SetNeutral(void);

// 读回右桥臂（EPWM1，GPIO0/1）当前 CMPA，调试观察用
Uint16 SPWM_GetCmp(void);

// 启停/故障封锁统一走 Trip Zone（Protect_ForceStop / Protect_Clear），
// 不要用 AQCSFRC 去封：AQCSFRC 只作用在 AQ 上，经死区子模块取反后
// EPWMxB 反而会变高，不是真正的四管全关

#endif
