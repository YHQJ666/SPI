/*
 * pr.c
 *
 * 准 PR 调节器：Tustin 离散系数设计 + 差分方程计算（全定点 Q24）
 *
 * 推导（k = 2/Ts，z^-1 为延迟算子）：
 *   G(s) = [ Kp*(s^2 + 2*wc*s + w0^2) + 2*Kr*wc*s ] / (s^2 + 2*wc*s + w0^2)
 * 令 s = k*(1-z^-1)/(1+z^-1)，分子分母同乘 (1+z^-1)^2：
 *   分母 = d0 + d1*z^-1 + d2*z^-2
 *     d0 = k^2 + 2*wc*k + w0^2
 *     d1 = -2*k^2 + 2*w0^2
 *     d2 = k^2 - 2*wc*k + w0^2
 *   分子 = n0 + n1*z^-1 + n2*z^-2
 *     n0 = Kp*d0 + 2*Kr*wc*k
 *     n1 = Kp*d1
 *     n2 = Kp*d2 - 2*Kr*wc*k
 * 两边同除 d0 得 a0 = 1 的形式
 *
 * 数值处理：k = 2/Ts = 40000（Ts=50us），k^2 = 1.6e9，
 * 直接算 d0/d1/d2 会出现大数相减(-2k^2 + 2w0^2)导致精度损失，定点更严重。
 * 这里对分子分母同除 k^2，用小量 r、q 计算：
 *   r  = w0/k = w0*Ts/2 = 0.007853982
 *   q  = wc/k = wc*Ts/2 = 0.00025
 *   d0'= 1 + 2q + r^2      ≈ 1.0005
 *   d1'= -2 + 2r^2         ≈ -1.99988
 *   d2'= 1 - 2q + r^2      ≈ 0.99956
 *   a1 = d1'/d0' , a2 = d2'/d0'
 *   kR = 2*Kr*q/d0'
 *   b0 = Kp + kR , b1 = Kp*a1 , b2 = Kp*a2 - kR
 *
 * r、q 由 dcac_cfg.h 用 _IQ(w0*Ts/2) 在编译期算好（Q24），
 * 所以本函数里只有定点加减乘除，没有任何浮点运算。
 * 除法用 _IQdiv：d1'/d0' 量级约 -2，d0' 归一化后接近 1，
 * 商没有超出 Q24 的动态范围，精度损失可以忽略。
 */

#include "pr.h"
// ★ 必须包含 dcac_cfg.h：下面的 #if (LOOP_MODE != LOOP_CURR_CLOSED) 要靠它拿宏。
//   只 include pr.h 的话两个宏都是未定义标识符、预处理按 0 处理，
//   于是 (0 != 0) 恒假，PR 在所有模式下都被编掉 —— 表现为
//   "只有电流单环能链接，mode 1/3 报 _PR_Calc / _PR_Design 未定义"。
#include "dcac_cfg.h"
#include "pid.h"            // PI_CLAMP：全工程共用一个钳位宏，不再另立 __clamp

// 全局 PR 结构体定义
PR_TypeDef PR_V;

/*===========================================================================
 * 函数名：PR_Reset
 * 说明：清掉全部历史状态（谐振腔 + 误差历史 + 积分），一行搞定。
 *       上电、清故障、切模式都调它 —— 以后再加状态也不会漏清。
 *       ★ 放在模式守卫外面：电流单环(2)虽然不用 PR，但那些分支里照样会
 *         把 PR 状态清一遍（死状态），这里必须能被所有模式调用。
 *===========================================================================*/
void PR_Reset(PR_TypeDef *p)
{
    p->Err      = 0;
    p->X1 = 0;  p->X2 = 0;
    p->Y1 = 0;  p->Y2 = 0;
    p->Integral = 0;
}

//===========================================================================
// ★ 只有存在电压环的模式才编译这两个函数。
//   LOOP_CURR_CLOSED(2) 里 PR 完全不用（分支里只是把历史清零），
//   把它编掉能省约 190 字 —— 28035 的 RAM 版代码区就 0x1A00 字，
//   电流环 + 完整 ISR + 浮点 RMS 已经把这块占满了。
//   切到 mode 1/3 时这里自动重新编进来，行为不变。
//===========================================================================
#if (LOOP_MODE != LOOP_CURR_CLOSED)

/*===========================================================================
 * 函数名：PR_Design
 * 说明：由 Kp/Kr/r/q 算出 5 个离散系数
 *       初始化时调用一次，之后中断里只跑 PR_Calc
 *===========================================================================*/
void PR_Design(PR_TypeDef *p, _iq Kp, _iq Kr, _iq KiTs, _iq r, _iq q)
{
    _iq r2;
    _iq d0p, d1p, d2p;
    _iq kR;

    p->Kp = Kp;
    p->Kr = Kr;
    p->KiTs = KiTs;         // 调用方已经在 dcac_cfg.h 里折成 Ki*Ts（Q24）
    p->Integral = 0;

    // 无量纲小量 r、q 由调用者按 Q24 传进来
    r2 = _IQmpy(r, r);                      // r^2，约 6.17e-5

    // 定点加减直接写，注意 1.0、2.0 都要用 _IQ() 转成 Q24
    d0p = _IQ(1.0f) + _IQmpy2(q) + r2;      // _IQmpy2(A) = A<<1，即乘 2
    d1p = _IQmpy2(r2) - _IQ(2.0f);
    d2p = _IQ(1.0f) - _IQmpy2(q) + r2;

    if (d0p == 0)                           // 理论上不会发生，兜底保护
    {
        p->b0 = 0; p->b1 = 0; p->b2 = 0;
        p->a1 = 0; p->a2 = 0;
        return;
    }

    p->a1 = _IQdiv(d1p, d0p);               // ≈ -1.99875
    p->a2 = _IQdiv(d2p, d0p);               // ≈ +0.99900

    // kR = 2*Kr*q/d0'，先乘后除，避免 q 太小被截断
    kR = _IQdiv(_IQmpy2(_IQmpy(Kr, q)), d0p);

    p->b0 = Kp + kR;
    p->b1 = _IQmpy(Kp, p->a1);
    p->b2 = _IQmpy(Kp, p->a2) - kR;
}

/* ★ 20kHz 中断里每拍调用（LOOP_MODE=1/3），必须放 RAM（理由见 pid.c 顶部）。
 *   电流单环（mode 2）时本函数整个被编掉，标在这里也只有 mode 1/3 生效。 */
#if (SFRA_FLASH_BUILD == 1)
#pragma CODE_SECTION(PR_Calc, "ramfuncs")
#endif

/*===========================================================================
 * 函数名：PR_Calc
 * 说明：准 PR 差分方程
 *       y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *       限幅后的输出回灌历史，防止谐振腔积分饱和
 *       5 个乘法全部走 _IQmpy（Q24 × Q24 -> Q24，64 位中间结果）
 *===========================================================================*/
_iq PR_Calc(PR_TypeDef *p)
{
    _iq y;
    _iq yr;
    _iq i_temp;
    _iq out_temp;

    p->Err = p->Ref - p->Fdb;

    // ---- 1. 比例 + 谐振（Kr=0 时这一整块恒等于 Kp*Err）----
    y = _IQmpy(p->b0, p->Err)
      + _IQmpy(p->b1, p->X1)
      + _IQmpy(p->b2, p->X2)
      - _IQmpy(p->a1, p->Y1)
      - _IQmpy(p->a2, p->Y2);

    // 谐振腔自己先限幅，并把限幅后的值回灌历史（原设计：防谐振腔积分饱和）
    yr = PI_CLAMP(y, p->OutMin, p->OutMax);

    p->X2 = p->X1;
    p->X1 = p->Err;
    p->Y2 = p->Y1;
    p->Y1 = yr;

    // ---- 2. 积分（Ki=0 时 i_temp 恒为 0，行为和以前的纯 PR 完全一致）----
    //     积分本身也限在输出范围内，单独也跑不飞
    i_temp = PI_CLAMP(p->Integral + _IQmpy(p->KiTs, p->Err),
                      p->OutMin, p->OutMax);

    // ---- 3. 抗饱和：候选输出饱和且误差同向，就不提交积分 ----
    out_temp = yr + i_temp;
    if (!((out_temp >= p->OutMax && p->Err > 0) ||
          (out_temp <= p->OutMin && p->Err < 0)))
    {
        p->Integral = i_temp;
    }

    p->Out = PI_CLAMP(yr + p->Integral, p->OutMin, p->OutMax);

    return p->Out;
}

/*===========================================================================
 * 函数名：PR_Sat
 * 说明：把 v 限在 [lo, hi]（Q24 定点，直接按整数比较）
 *===========================================================================*/
_iq PR_Sat(_iq v, _iq lo, _iq hi)
{
    return PI_CLAMP(v, lo, hi);
}

#endif  /* LOOP_MODE != LOOP_CURR_CLOSED : 电流单环下 PR 整个编掉 */
