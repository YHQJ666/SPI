/*
 * protect.c
 *
 * 纯软件保护实现（欠压 + 过流），详见 protect.h 的说明。
 *
 * 本文件完全不碰 COMP1Regs / Comp2Regs / TZDCSEL / DCACTL / DCBCTL：
 * 板上没有比较器通路，Digital Compare 的事件源一律不使能。
 */

#include "protect.h"
#include "dcac_cfg.h"

volatile Uint16 Protect_UvFault = 0U;
volatile Uint16 Protect_OcFault = 0U;
volatile Uint16 Protect_UvCnt   = 0U;
volatile Uint16 Protect_UvClrCnt = 0U;
volatile Uint16 Protect_OcCnt   = 0U;

/*===========================================================================
 * 函数名：Protect_ConfigTz
 * 说明：把一个 ePWM 模块的触发区配成"只能软件触发、触发即四管全关"
 *       TZCTL 复位值是 0xFFFF（强制输出高），必须显式写成 force-low
 *===========================================================================*/
static void Protect_ConfigTz(volatile struct EPWM_REGS *epwm)
{
    // A/B 输出在单次/逐周期触发时都拉低
    epwm->TZCTL.bit.TZA = TZ_FORCE_LO;
    epwm->TZCTL.bit.TZB = TZ_FORCE_LO;
    // Digital Compare 的四个事件通道也一并拉低（虽然本工程不产生它们）
    epwm->TZCTL.bit.DCAEVT1 = TZ_FORCE_LO;
    epwm->TZCTL.bit.DCAEVT2 = TZ_FORCE_LO;
    epwm->TZCTL.bit.DCBEVT1 = TZ_FORCE_LO;
    epwm->TZCTL.bit.DCBEVT2 = TZ_FORCE_LO;

    // 外部触发源只选单次（OSHT1）。逐周期 CBC 和 Digital Compare 事件全不选。
    // ★ 不要把 TZSEL 整个清零：spec 要求 OSHT=1，清零会把 OSHT1..6 一起清掉。
    //   TZSEL 位定义（DSP2803x_EPwm.h）：bit0-5 = CBC1..6，bit6/7 = DCAEVT2/DCBEVT2，
    //   bit8-13 = OSHT1..OSHT6。写 0x0100 = 只有 OSHT1。
    epwm->TZSEL.all = 0x0100U;

    // 不开 TZ 中断（软件封波不需要中断）
    epwm->TZEINT.all = 0x0000U;

    // 清掉可能残留的触发标志
    epwm->TZCLR.all = 0xFFFFU;
}

/*===========================================================================
 * 函数名：Protect_Init
 * 说明：无条件执行，与 Protect_Enable 无关（见 protect.h 的说明）
 *===========================================================================*/
void Protect_Init(void)
{
    Protect_UvFault = 0U;
    Protect_OcFault = 0U;
    Protect_UvCnt   = 0U;
    Protect_OcCnt   = 0U;

    EALLOW;
    Protect_ConfigTz(&EPwm1Regs);
    Protect_ConfigTz(&EPwm2Regs);
    EDIS;
}

/*===========================================================================
 * 函数名：Protect_Update
 * 说明：20kHz 中断里每拍调用。Q24 定点比较就是整数比较，不做任何转换
 *       欠压不锁存（电压回来自动恢复），过流锁存
 *===========================================================================*/
/* ★ 20kHz 中断里每拍调用，必须放 RAM（理由见 pid.c 顶部） */
#if (SFRA_FLASH_BUILD == 1)
#pragma CODE_SECTION(Protect_Update, "ramfuncs")
#endif

void Protect_Update(_iq vin, _iq il)
{
    // ---- 欠压：进入要连续 UV_Debounce 拍，退出也要连续 UV_Clear_Debounce 拍 ----
    // ★ 退出没有去抖的话：母线在 55V 上下抖一格，标志就掉一下，
    //   main.c 那边 UvRamp 会被清零重爬，兜底输出直接塌掉（评审指出）
    if (vin < UV_Volts)
    {
        if (Protect_UvCnt < UV_Debounce)
        {
            Protect_UvCnt++;
        }
        Protect_UvClrCnt = 0U;
    }
    else
    {
        if (Protect_UvClrCnt < UV_Clear_Debounce)
        {
            Protect_UvClrCnt++;
        }

        if (Protect_UvClrCnt >= UV_Clear_Debounce)
        {
            Protect_UvCnt = 0U;
        }
    }

    Protect_UvFault = (Protect_UvCnt >= UV_Debounce) ? 1U : 0U;

    // ---- 过流：绝对值比较，连续 OC_Debounce 拍才认，锁存 ----
    if ((il > OC_Amps) || (il < -OC_Amps))
    {
        if (Protect_OcCnt < OC_Debounce)
        {
            Protect_OcCnt++;
        }
    }
    else
    {
        Protect_OcCnt = 0U;
    }

    if (Protect_OcCnt >= OC_Debounce)
    {
        Protect_OcFault = 1U;
    }

    // ---- 动作：只在 Protect_Enable==1 时封波 ----
    // 欠压的动作不在这里：欠压要切固定 Mod_Max 正弦兜底，属于控制环，见 main.c
#if (Protect_Enable == 1)
    if (Protect_OcFault != 0U)
    {
        Protect_ForceStop();
    }
#endif
}

/*===========================================================================
 * 函数名：Protect_Clear
 * 说明：清过流锁存并清 TZ 触发，允许重新输出
 *===========================================================================*/
void Protect_Clear(void)
{
    Protect_OcFault = 0U;
    Protect_OcCnt   = 0U;

    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm1Regs.TZCLR.bit.INT = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.INT = 1;
    EDIS;
}

/*===========================================================================
 * 函数名：Protect_ForceStop
 * 说明：软件封波。置 TZFRC.OST 后立刻清中断标志，所以不会产生 TZ 中断
 *       注意：即使 Protect_Enable==0 也真的会封波 —— 这是"主动急停"，
 *       只有 Protect_Update 里的自动判断才受 Protect_Enable 约束
 *===========================================================================*/
void Protect_ForceStop(void)
{
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm1Regs.TZCLR.bit.INT = 1;
    EPwm2Regs.TZCLR.bit.INT = 1;
    EDIS;
}
