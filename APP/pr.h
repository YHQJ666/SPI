/*
 * pr.h
 *
 * 准比例谐振(PR) + 积分：Tustin 离散系数设计 + 差分方程计算
 *
 * 连续域：
 *                 2*wc*s
 *   G(s) = Kp + Kr*------------------------ + Ki/s
 *                s^2 + 2*wc*s + w0^2
 *          ^^   ^^^^^^^^^^^^^^^^^^^^^^^^^^   ^^^^
 *          比例         准谐振               积分
 *
 * ★★ 不做任何宏切换，三种形态全靠增益取 0（就是"配哪个就用哪个"）：
 *     KR_V = 0            -> 纯 PI
 *     KI_V = 0            -> 纯 PR（积分不动作，行为和以前完全一样）
 *     两个都不为 0         -> PI + PR
 *
 *   为什么 KR=0 就退化成纯 P：Kr=0 时 kR=0、b0=Kp、b1=Kp*a1、b2=Kp*a2，
 *   于是 (b0+b1*z^-1+b2*z^-2)/(1+a1*z^-1+a2*z^-2) = Kp*(...)/(...) = Kp，
 *   二阶环节的零极点正好约掉，剩一个纯增益。再叠上 Ki/s 就是标准 PI。
 *
 * ★ 交流运行（50Hz 正弦给定）**必须** KR_V != 0：
 *   PI 对正弦只能给出有限增益，稳态必然留一个明显静差；跟踪 50Hz 靠 KR。
 *   KI_V 在交流下只干一件事 —— 清直流偏置/消慢慢漂的静差。
 *   直流工作点（SFRA 的 DC_CHECK、或纯 DC 输出）反过来：KI 有用，KR 没用。
 *
 * Tustin 离散（Ts = 1/fs）后：
 *   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *   + 积分累加（Ki/s 用矩形积分，KiTs = Ki*Ts 已经折好）
 *
 * 定点化（GLOBAL_Q = 24）：
 *   系数 b0/b1/b2/a1/a2 都是 ±2 以内的小数，Q24 精度足够；
 *   运行量 x = 电压误差 ±51V、y = 电感电流给定 ±8A，都在 Q24 的 ±128 以内；
 *   a1 ≈ -1.9988、y1 最大 8A，a1*y1 ≈ 16，仍在 ±128 以内，
 *   而且 _IQmpy 用 64 位中间结果，不会溢出。
 *
 * 用法和 pid.c 的 PI_Calc 一样：结构体 + Calc 函数
 *   PR_V.Ref = 给定; PR_V.Fdb = 反馈; i_ref = PR_Calc(&PR_V);
 */

#ifndef PR_H
#define PR_H

#include "IQmathLib.h"

// 准 PR 结构体，全部 Q24
typedef struct
{
    // 设计输入
    _iq Kp;           // 比例增益
    _iq Kr;           // 谐振增益（=0 时谐振支路退化成纯增益 Kp，即纯 P）
    _iq Ki;           // 积分增益 [1/s]（=0 时不加积分，即纯 PR）
    _iq KiTs;         // = Ki*Ts，已经折好，中断里只乘一次

    // Tustin 离散系数（a0 归一化为 1），Q24
    // w0、wc、Ts 在 PR_Design 之前就已经合成 r = w0*Ts/2、q = wc*Ts/2，
    // 由 dcac_cfg.h 按编译期常量算好，运行期不保留这三个量
    _iq b0;
    _iq b1;
    _iq b2;
    _iq a1;
    _iq a2;

    // 运行量
    _iq Ref;        // 设定值
    _iq Fdb;        // 反馈值
    _iq Err;        // 误差 = Ref - Fdb
    _iq X1;         // 误差历史 x[n-1]
    _iq X2;         // 误差历史 x[n-2]
    _iq Y1;         // 输出历史 y[n-1]
    _iq Y2;         // 输出历史 y[n-2]
    _iq Integral;   // 积分累加器（Ki=0 时恒为 0，不影响原来纯 PR 的行为）
    _iq Out;        // 输出
    _iq OutMax;     // 输出上限（积分也被限在这个范围内）
    _iq OutMin;     // 输出下限
} PR_TypeDef;

// 全局 PR 结构体实例
extern PR_TypeDef PR_V;     // 电压外环

// 按 Tustin 法计算 b0/b1/b2/a1/a2，并把 Ki 折成 KiTs
//   r = w0*Ts/2, q = wc*Ts/2，都是 Q24（w0 超出 Q24 范围，
//   已经在 dcac_cfg.h 里折成 r、q，不会再出现 314 这种数）
//   ★ Ki 传 0 就是不加积分（纯 PR）；Kr 传 0 就是纯 P（退化后正好是 Kp）
void PR_Design(PR_TypeDef *p, _iq Kp, _iq Kr, _iq Ki, _iq r, _iq q);

// 清掉全部历史：谐振腔 + 误差历史 + 积分
// 上电、清故障、切模式时一行搞定，不会漏掉新加的状态
void PR_Reset(PR_TypeDef *p);

// 差分方程计算，返回限幅后的输出
_iq PR_Calc(PR_TypeDef *p);

// 两个 Q24 限幅
_iq PR_Sat(_iq v, _iq lo, _iq hi);

#endif
