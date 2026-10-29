/*
 * oled_task.c
 *
 * OLED 显示任务（0.96 寸 SSD1306，I2C 位带，SCL=GPIO29 / SDA=GPIO28）。
 * 从 main.c 搬出来的：它只在后台每 500ms 刷一次屏，和 20kHz 控制环无关。
 *
 * 数据来源：
 *   有效值/校准值 -> Debug_View（debug_view.c）
 *   故障状态      -> 参数 fault 传入（main.c 根据 protect.c 的标志算出来的）
 *
 * 刷一屏要几十毫秒（位带 I2C，没有硬件外设），所以必须节流，
 * 用中断计数做时基，不依赖 CPU 定时器。
 */

#include "oled_task.h"
#include "Oled.h"
#include "debug_view.h"
#include "protect.h"
#include "dcac_cfg.h"

#if (SFRA_ENABLE == 0)
//===========================================================================
// OLED 显示（0.96 寸 SSD1306，I2C 位带，SCL=GPIO29 / SDA=GPIO28）
//
// 屏是 128x64，用 6x8 点阵（Oled.c 里 Char_Size=12 走 F6x8）：
//   横向 128/8 = 16 个字符（ShowString 每个字符前进 8 像素）
//   纵向 64/8  = 8 行（y 就是页号 0~7，一行一页）
//
// 显示内容（8 行正好占满）：
//   0  ST:OK        保护状态
//   1  LOOP:CLOSED  开环 / 闭环
//   2  Mod:0.870    实际调制比幅值（稳态值，不是瞬时调制比）
//   3  Vo:33.4 Zc:1.51   Vout 有效值 + 零点校准值（同一行）
//   4  Io:3.31      输出电流有效值
//   5  IL:3.40      电感电流有效值
//   6  Vi:48.2      输入电压
//   7  Ii:0.35      输入电流
//
// 刷新在后台做，每 Oled_Period_Isr 个中断（500ms）刷一次。
// 刷一屏要几十毫秒（I2C 位带，没有硬件外设），所以不能刷太快。
//===========================================================================
#define Oled_Period_Isr   10000U

static Uint16 OledLastIsr = 0U;

// 定宽显示浮点数：ints 位整数 + decs 位小数，高位 0 用空格代替。
// 不用 OLED_ShowNum 是因为它固定打小数点、而且负数会出错。
static void Oled_F(Uint16 x, Uint16 y, float v, Uint16 ints, Uint16 decs)
{
    char buf[12];
    u32  s;
    u32  div;
    u32  ip;
    u32  fp;
    Uint16 i;
    Uint16 k;
    Uint16 n;
    Uint16 started;

    if (v < 0.0f)
    {
        v = -v;
    }

    for (i = 0U; i < decs; i++)
    {
        v *= 10.0f;
    }

    if (v > 9999999.0f)
    {
        v = 9999999.0f;
    }

    s = (u32)(v + 0.5f);

    div = 1U;
    for (i = 0U; i < decs; i++)
    {
        div *= 10U;
    }

    ip = s / div;
    fp = s % div;

    n = 0U;
    started = 0U;

    // 整数部分：从最高位往下打，前导 0 变空格
    for (i = ints; i > 0U; i--)
    {
        u32 d = 1U;
        Uint16 dig;

        for (k = 0U; k < (Uint16)(i - 1U); k++)
        {
            d *= 10U;
        }

        dig = (Uint16)((ip / d) % 10U);

        if ((dig != 0U) || (started != 0U) || (i == 1U))
        {
            started = 1U;
            buf[n] = (char)('0' + (char)dig);
        }
        else
        {
            buf[n] = ' ';
        }
        n++;
    }

    // 小数部分
    if (decs > 0U)
    {
        buf[n] = '.';
        n++;

        for (i = decs; i > 0U; i--)
        {
            u32 d = 1U;

            for (k = 0U; k < (Uint16)(i - 1U); k++)
            {
                d *= 10U;
            }

            buf[n] = (char)('0' + (char)((fp / d) % 10U));
            n++;
        }
    }

    buf[n] = '\0';
    OLED_ShowString(x, y, (u8 *)buf);
}

static void Oled_Draw(Uint16 fault)
{
    // 第 0 行：保护状态 + 开环/闭环（两个都很短，合在一行）
    //   S:OK  L:CLOSE  ->  保护正常、闭环
    //   S:OK  L:OPEN   ->  保护正常、开环
    //   S:UV           ->  输入欠压（不锁存，电压回来自动消失）
    //   S:OC           ->  电感过流（锁存，要 FaultClear）
    if (fault == FAULT_OC)
    {
        OLED_ShowString(0U, 0U, (u8 *)"S:OC ");
    }
    else if (fault == FAULT_UV)
    {
        OLED_ShowString(0U, 0U, (u8 *)"S:UV ");
    }
    else
    {
        OLED_ShowString(0U, 0U, (u8 *)"S:OK ");
    }

#if (LOOP_MODE != LOOP_SINE_OPEN)
    OLED_ShowString(48U, 0U, (u8 *)"L:CLOSE");
#else
    OLED_ShowString(48U, 0U, (u8 *)"L:OPEN");
#endif

    // 第 1 行留空
    // 第 2 行：调制比幅值（开环固定 Mod_Amp_Open；闭环因电感压降会略大）
    OLED_ShowString(0U, 2U, (u8 *)"M:");
    Oled_F(16U, 2U, Debug_View.ModAmp, 1U, 3U);

    // 第 3 行留空
    // 第 4 行：Vout 有效值 + 单位 + 零点校准值（校准值不写单位）
    OLED_ShowString(0U, 4U, (u8 *)"VO:");
    Oled_F(24U, 4U, Debug_View.VoutRms, 2U, 1U);
    OLED_ShowString(56U, 4U, (u8 *)"V");
    OLED_ShowString(80U, 4U, (u8 *)"Z:");
    Oled_F(96U, 4U, Debug_View.CalPinVout, 1U, 2U);

    // 第 5 行：Iout 有效值 + 单位 + 零点校准值
    OLED_ShowString(0U, 5U, (u8 *)"IO:");
    Oled_F(24U, 5U, Debug_View.IoutRms, 1U, 2U);
    OLED_ShowString(56U, 5U, (u8 *)"A");
    OLED_ShowString(80U, 5U, (u8 *)"Z:");
    Oled_F(96U, 5U, Debug_View.CalPinIout, 1U, 2U);

    // 第 6 行：IL 有效值 + 单位 + 零点校准值
    OLED_ShowString(0U, 6U, (u8 *)"IL:");
    Oled_F(24U, 6U, Debug_View.ILRms, 1U, 2U);
    OLED_ShowString(56U, 6U, (u8 *)"A");
    OLED_ShowString(80U, 6U, (u8 *)"Z:");
    Oled_F(96U, 6U, Debug_View.CalPinIL, 1U, 2U);

    // 第 7 行：输入电压（输入电流不显示了）
    OLED_ShowString(0U, 7U, (u8 *)"VIN:");
    Oled_F(32U, 7U, Debug_View.Vin, 2U, 1U);
    OLED_ShowString(64U, 7U, (u8 *)"V");
}

/*===========================================================================
 * 函数名：Oled_Task_Init
 * 说明：打开 OLED 并清屏。要在 ePWM / 中断启动之前调用。
 *===========================================================================*/
void Oled_Task_Init(void)
{
    OLED_Init();
    OLED_Clear();
}

/*===========================================================================
 * 函数名：Oled_Task_Run
 * 说明：后台循环里调用，内部节流。
 *       每 Oled_Period_Isr 个中断（10000 拍 @20kHz = 500ms）刷一屏。
 *       用中断计数差做时基，无符号相减自动处理回绕。
 *===========================================================================*/
void Oled_Task_Run(Uint16 isrcnt, Uint16 fault)
{
    if ((Uint16)(isrcnt - OledLastIsr) >= Oled_Period_Isr)
    {
        OledLastIsr = isrcnt;
        Oled_Draw(fault);
    }
}

#else   /* SFRA_ENABLE == 1 : 扫频版没有 OLED，两个函数退化成空 */

void Oled_Task_Init(void)
{
}

void Oled_Task_Run(Uint16 isrcnt, Uint16 fault)
{
    (void)isrcnt;
    (void)fault;
}

#endif  /* SFRA_ENABLE */
