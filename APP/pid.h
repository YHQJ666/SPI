#ifndef _PID_H
#define _PID_H

#include "IQmathLib.h"

// PI控制器结构体，全部定点 Q24（GLOBAL_Q = 24）
// 量级检查（Q24 只有 ±128，所有中间量都必须落在里面）：
//   比例项 p_term = Kp*err：
//     电压环 KP_V = 0.0518，误差最大 51V        -> 2.6
//     电流环 KP_I = 12.533，误差 8A             -> 100.3   ← 已贴近 ±128
//     ★ 电流环 |err| 一旦超过 10.2A，p_term 就会溢出 Q24：
//       所以 Kp 本来就压不到这个量级（设计上限约 3 V/A，见 dcac_cfg.h）
typedef struct
{
    _iq Ref;        // 设定值(目标)
    _iq Fdb;        // 反馈值(采样)
    _iq Err;        // 误差 = Ref - Fdb
    _iq ErrMax;     // ★ 误差前置限幅（幅值）。0 = 不限。
                    //   Q24 边界 ±128，p_term = Kp*err 必须留在里面，否则 int32 翻转
                    //   符号、PI 反向打。Kp=11.74 时 |err| 上限只有 10.9A，
                    //   而过流阈值是 12A —— 必须在这里先限住
    _iq Kp;         // 比例系数
    _iq KiTs;       // 积分增益，已经把采样周期折进去（Ki*Ts），中断里只乘一次
    _iq Integral;   // 积分累加器
    _iq Out;        // PI输出
    _iq OutMax;     // 输出上限
    _iq OutMin;     // 输出下限
    _iq IntMax;     // 积分上限
    _iq IntMin;     // 积分下限
}PI_TypeDef;

// 全局PI结构体实例
extern PI_TypeDef PI_V;  // 备用，本工程电压外环用准PR，见 pr.h
extern PI_TypeDef PI_I;  // 电流内环（本工程反馈用**电感电流 i_L**，不是电容电流 i_c）

//===========================================================================
// 限幅工具：全工程只留这两样，主循环里的限幅一律写成一行
//   PI_CLAMP(v,lo,hi)          —— 钳位，替换原来 pid.c 里私有的 __clamp
//   PI_SetLimits(pi,lo,hi)     —— 一次性把输出和积分四个边界都设好
//                                  （OutMin/OutMax/IntMin/IntMax 一起设，
//                                    免得漏设某一个导致 __clamp 拿到乱序边界）
//===========================================================================
#define PI_CLAMP(v, lo, hi) \
    ((v) > (hi) ? (hi) : ((v) < (lo) ? (lo) : (v)))

extern void PI_SetLimits(PI_TypeDef *pi, _iq lo, _iq hi);

// 函数声明
extern _iq PI_Calc(PI_TypeDef *pi);

#endif // _PID_H
