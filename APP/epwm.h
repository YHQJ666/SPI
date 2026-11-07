#ifndef EPWM_H
#define EPWM_H

#include "DSP28x_Project.h"  // 引入Uint16等类型定义

// 1. GPIO初始化函数声明
void InitEPwmGpioAll(void);
// 只把 GPIO0~GPIO3 复用成 EPWM1A/1B、EPWM2A/2B（spwm.c 用这个，不是上面那个全量版）
void InitPwmBridgeGpio(void);

// 2. EPWM初始化函数声明
// 主模块（EPWM1）：无相位偏移参数
void EPWM1_Init(Uint16 tbprd, Uint16 dbRed, Uint16 dbFed);
// 从模块（EPWM2~EPWM6）：带相位偏移参数
void EPWM2_Init(Uint16 tbprd, Uint16 dbRed, Uint16 dbFed, Uint16 phaseShift);
void EPWM3_Init(Uint16 tbprd, Uint16 dbRed, Uint16 dbFed, Uint16 phaseShift);
void EPWM4_Init(Uint16 tbprd, Uint16 dbRed, Uint16 dbFed, Uint16 phaseShift);
void EPWM5_Init(Uint16 tbprd, Uint16 dbRed, Uint16 dbFed, Uint16 phaseShift);
void EPWM6_Init(Uint16 tbprd, Uint16 dbRed, Uint16 dbFed, Uint16 phaseShift);

// 3. 占空比设置函数声明（A通道）
void EPwm1A_SetCompare(Uint16 val);
void EPwm2A_SetCompare(Uint16 val);
void EPwm3A_SetCompare(Uint16 val);
void EPwm4A_SetCompare(Uint16 val);
void EPwm5A_SetCompare(Uint16 val);
void EPwm6A_SetCompare(Uint16 val);

#endif  // EPWM_H

//使用实例
//// PWM 参数
//Uint16 tbprd  = 500;  // 100kHz
//Uint16 dbRed  = 10;   // 死区 200ns
//Uint16 dbFed  = 10;
//
//// 相位值（60° 间隔）
//Uint16 phase2 = 83;   // 60°
//Uint16 phase3 = 167;  // 120°
//Uint16 phase4 = 250;  // 180°
//Uint16 phase5 = 333;  // 240°
//Uint16 phase6 = 417;  // 300°
//
//// 初始化 GPIO
//InitEPwmGpioAll();
//
//// 关 TBCLKSYNC
//EALLOW;
//SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
//EDIS;
//
//// 初始化 EPWM
//EPWM1_Init(tbprd, dbRed, dbFed);             // 主
//EPWM2_Init(tbprd, dbRed, dbFed, phase2);     // 从
//EPWM3_Init(tbprd, dbRed, dbFed, phase3);
//EPWM4_Init(tbprd, dbRed, dbFed, phase4);
//EPWM5_Init(tbprd, dbRed, dbFed, phase5);
//EPWM6_Init(tbprd, dbRed, dbFed, phase6);
//
//// 占空比 50%
//EPwm1A_SetCompare(250);
//EPwm2A_SetCompare(250);
//EPwm3A_SetCompare(250);
//EPwm4A_SetCompare(250);
//EPwm5A_SetCompare(250);
//EPwm6A_SetCompare(250);
//
//// 开 TBCLKSYNC
//EALLOW;
//SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
//EDIS;
