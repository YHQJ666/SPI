/*
 * pid.c
 *
 * 增量式带抗饱和的 PI 调节器（全定点 Q24）
 * 写法沿用 Boost 工程：结构体 + PI_Calc 函数
 *
 * 积分项的定点处理：
 *   原来的写法是 Ki * err * Ts，每拍乘两次。
 *   定点下更划算也更准的做法是把采样周期折进系数：KiTs = Ki*Ts，
 *   初始化时由 dcac_cfg.h 的 KI_I_Ts 算好（编译期常量），
 *   中断里只做一次 _IQmpy(pi->KiTs, err)。
 *   这样做还避免了 err*Ts 先算出来太小的问题：Ts = 50us 在 Q24 里只有
 *   838 个刻度，先乘 Ts 会把误差的有效位数吃掉十几位。
 *
 * 采样周期仍然来自 dcac_cfg.h 的 Ts_S（20kHz -> 50us），
 * 换载波频率时 Ki*Ts 会自动跟着变，不用手改
 */

#include "pid.h"
#include "dcac_cfg.h"

/* ★ 20kHz 中断里每拍都调这两个函数，必须放 RAM。
 *   Flash 在 60MHz 下 RANDWAIT=PAGEWAIT=2（DSP2803x_SysCtrl.c 里设的），
 *   一次取指 3 个周期，而**函数调用会打断 flash 预取流水线**——
 *   中断预算只有 50us = 3000 个 CPU 周期，这点代价足以把闭环顶超时。
 *   RAM 版（SFRA_FLASH_BUILD=0）里代码本来就在 RAM，这里不用管。 */
#if (SFRA_FLASH_BUILD == 1)
#pragma CODE_SECTION(PI_SetLimits, "ramfuncs")
#pragma CODE_SECTION(PI_Calc,      "ramfuncs")
#endif

// 全局 PI 结构体定义
PI_TypeDef PI_V;
PI_TypeDef PI_I;

// 限幅宏 PI_CLAMP 在 pid.h 里（全工程共用一份，不在本文件另立私有的）

/*===========================================================================
 * 函数名：PI_SetLimits
 * 说明：一次把输出限幅和积分限幅四个边界都设好，调用处就一行
 *         PI_SetLimits(&PI_I, -V_L_Max, V_L_Max);
 *       原来要写四行，而且容易漏设 —— 漏了就会让 PI_CLAMP 拿到
 *       OutMax < OutMin 这种乱序边界，PI 输出直接被钉成常数
 *===========================================================================*/
void PI_SetLimits(PI_TypeDef *pi, _iq lo, _iq hi)
{
    pi->OutMin = lo;
    pi->OutMax = hi;
    pi->IntMin = lo;
    pi->IntMax = hi;
}

/*===========================================================================
 * 函数名：PI_Calc
 * 说明：PI 计算核心。先算候选积分和候选输出，只有输出没饱和才提交积分
 *       乘法全部用 _IQmpy，直接写 a*b 得到的是右移 24 位之前的 Q48 值
 *===========================================================================*/
_iq PI_Calc(PI_TypeDef *pi)
{
    _iq err = pi->Ref - pi->Fdb;                    // 定点加减直接用整数运算

    // ★ 误差必须在进乘法之前限住：
    //   Q24 的边界是 ±128，_IQmpy 在 C28x 上是 32 位结果、**不饱和**，
    //   p_term = Kp*err 一旦越界（128*2^24 正好是 int32 边界），符号直接翻转：
    //   本该 +127 的项变成 -128，PI 反向打，现场表现就是"闭环一跑就飞"。
    //   Kp=11.74 时 |err| 只到 10.9A 就越界，而过流阈值是 12A ——
    //   10.9~12A 这一段是"既没保护、又算错"的死区。
    //   ErrMax = 0 表示不限（没设过的实例不会被误限成 0）
    if (pi->ErrMax > 0)
    {
        err = PI_CLAMP(err, -pi->ErrMax, pi->ErrMax);
    }

    _iq p_term = _IQmpy(pi->Kp, err);               // 比例项

    // 候选积分：KiTs 已经含采样周期，这里只乘一次
    _iq i_temp = pi->Integral + _IQmpy(pi->KiTs, err);
    i_temp = PI_CLAMP(i_temp, pi->IntMin, pi->IntMax);

    // 候选未限幅输出
    _iq out_temp = p_term + i_temp;

    // 抗饱和：候选输出饱和且误差同向，则不更新积分
    if (!((out_temp >= pi->OutMax && err > 0) ||
          (out_temp <= pi->OutMin && err < 0)))
    {
        pi->Integral = i_temp;
    }

    pi->Err = err;

    pi->Out = p_term + pi->Integral;
    pi->Out = PI_CLAMP(pi->Out, pi->OutMin, pi->OutMax);

    return pi->Out;
}
