# 电流环前馈 + 保护简化 + main.c 拆分 实施方案

> **For agentic workers:** REQUIRED SUB-SKILL: Use subagent-driven-development (recommended) or executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给单电感电流环加上"输出电压 + 母线电压"双前馈（结构照 TI 逆变器例程式 12），把硬件保护换成两条纯软件判断，并把 main.c 里的调试/显示/扫频三块拆成独立文件。

**Architecture:** 电流环 PI 的输出量纲从"调制比"改成"电压"，再用 `mod = (PI + VoutFF) × (1/Vin)` 得到占空比；`1/Vin` 在后台每 20ms 算一次，ISR 里只做乘法。保护模块只保留 `Vin < 55V` 和 `|IL| > 12A` 两条软件判断，`Protect_Enable = 0` 时只置标志不动作。调试镜像 / OLED / SFRA 各自成文件，通过窄接口和 main.c 交互。

**Tech Stack:** TI C2000 (F28035)，C，TI IQmath 定点（`GLOBAL_Q = 24`），CCS10.4 / `ti-cgt-c2000_20.2.5.LTS`，`gmake` + 28035 RAM 链接脚本。

**Spec:** `docs/superpowers/specs/2026-10-29-current-loop-ff-protect-refactor-design.md`

## Global Constraints

- 20kHz ADC 中断里**不得出现浮点运算**，也不得出现除法（`_IQdiv` / `F$$DIV` 都不行）。每次涉及 ISR 的任务都要用反汇编复核。
- `GLOBAL_Q = 24`，动态范围 ±128。所有新的 Q24 常量必须在 ±128 以内。
- 编译命令（在 `Debug\` 目录下，`Debug\` 已被 .gitignore 忽略，可随意改）：
  ```powershell
  $env:PATH = "C:\ti\ccs1040\ccs\utils\bin;" + $env:PATH
  & "C:\ti\ccs1040\ccs\utils\bin\gmake.exe" -f makefile -j4 all
  ```
  成功判据：`EXIT=0` 且出现 `Finished building target: "SPI_LC_iq.out"`。
- 允许的 warning 基线（改前就存在，不要新增）：`protect.c` 未引用函数 2 条、`spwm.c` 隐式声明 1 条。`sin_val` / `sin_table` 未引用两条是**当前被注释的 ISR 造成的临时 warning，恢复 ISR 主体后应消失**。
- 新增 .c 文件后，CCS 生成的 `Debug\makefile` / `Debug\subdir_vars.mk` 不认识它们。命令行验证时手动把 `./APP/xxx.obj` 加进 `Debug\makefile` 的 `ORDERED_OBJS` 和链接命令行（这两个文件是生成物、不入库）。
- 所有开关集中在 `APP/dcac_cfg.h` 顶部"常改区"。
- **开工前置条件**：当前工作区里 `main.c` 的 `ADC_ISR` 主体（采样换算 → 控制环 → 写 CMPA）被用户临时注释掉了。**实现开始前必须先恢复**（用 `git diff main.c` 对照，或从 `git show HEAD:main.c` 取回那一段），否则后面所有任务都建立在空中断上。

## Review Focus

这些是文档没写、但一定会被现场遇到的输入，每个都必须在对应任务里落成代码保护：

1. **上电瞬间 `Vin ≈ 0`**：`InvVin = 1/Vin` 会爆掉 → 必须钳到 `Vin_Ff_Min`，且 `InvVin` 在开中断前初始化成 `1/Vbus_Nominal`。
2. **母线过压（`Vin > 90V`）**：`1/Vin` 变得过小，前馈量不足 → 钳到 `Vin_Ff_Max`。
3. **母线跌落但输出电压还高（`Vout > 0.85·Vin`）**：动态限幅的 `head` 变负，`OutMax < OutMin` → `head` 必须钳到 ≥ 0。
4. **`Vin` 在 55V 上下抖动**：欠压标志在 20kHz 下反复翻转，兜底和闭环来回切 → 去抖计数 + 进入时重置斜坡。
5. **`Vout` 采样通道没接/校零失败**：`VoutFF = 0` 时前馈退化成纯 PI，仍应能运行（不能崩）；`CalPin*` 异常只影响 OLED 显示。

---

### Task 1: 配置区重构 + 软件保护模块

**Files:**
- Modify: `APP/dcac_cfg.h`（顶部加"常改区"；删 `Use_Comp2_Negative` 等硬件保护相关宏）
- Rewrite: `APP/protect.c`、`APP/protect.h`
- Modify: `main.c`（删 `Epwm1_TripIsr`、删 `PieVectTable.EPWM1_TZINT`、删 ISR 里的内联过流/过压判断、`Protect_InitOCP()` → `Protect_Init()`）

**Interfaces:**
- Produces:
  ```c
  extern volatile Uint16 Protect_UvFault;   /* 1 = Vin < UV_Volts */
  extern volatile Uint16 Protect_OcFault;   /* 1 = |IL| > OC_Amps，锁存 */
  void Protect_Init(void);                  /* 配 TZ 软件封波（TZCTL force-low, TZSEL.OSHT=1, 不开 TZEINT） */
  void Protect_Update(_iq vin, _iq il);     /* ISR 每拍调用，参数是原始 Vin 和 IL（Q24） */
  void Protect_Clear(void);                 /* 清锁存 + 清 TZ */
  void Protect_ForceStop(void);             /* TZFRC.OST = 1，四管全关 */
  ```
- 配置宏（写进 `dcac_cfg.h` 常改区）：
  ```c
  #define UV_Volts_F      55.0f
  #define OC_Amps_F       12.0f
  #define UV_Debounce     200U      /* 10ms @20kHz */
  #define OC_Debounce     3U
  #define Protect_Enable  0         /* 0 = 只判断不动作 */
  #define UV_Volts        _IQ(UV_Volts_F)
  #define OC_Amps         _IQ(OC_Amps_F)
  ```

- [ ] **Step 1: 恢复被注释的 ISR 主体**

跑 `git diff main.c`，把 `ADC_ISR` 里从"1. 采样并换算工程量"到 `SPWM_SetMod(Mod)` 的整段取消注释。用 `git stash` 前后对比确认恢复后的 `main.c` 与我上一轮改完的状态一致（`Iref_Peak` 软启动、`Iref_Max` 限幅、`CpuTimer0` 时间戳都在）。编译通过。

- [ ] **Step 2: 写保护的失败判据（先写"测试"）**

在 `APP/protect.h` 里定义接口、`APP/protect.c` 里先写空实现（`Protect_Update` 直接返回），`main.c` 里接上调用。编译。此时 `Protect_UvFault` / `Protect_OcFault` 恒 0 —— 这就是"测试失败"状态。

- [ ] **Step 3: 实现 `Protect_Update`**

判据与去抖（参照 Review Focus 4）：

```c
void Protect_Update(_iq vin, _iq il)
{
    /* 欠压：去抖，不锁存 */
    if (vin < UV_Volts) { if (UvCnt < UV_Debounce) UvCnt++; }
    else                { UvCnt = 0U; UvEntered = 0U; }
    Protect_UvFault = (UvCnt >= UV_Debounce) ? 1U : 0U;

    /* 过流：去抖 + 锁存（绝对值比较，整数比较，不用 IQmpy） */
    if ((il > OC_Amps) || (il < -OC_Amps)) { if (OcCnt < OC_Debounce) OcCnt++; }
    else                                   { OcCnt = 0U; }
    if (OcCnt >= OC_Debounce) { Protect_OcFault = 1U; }

    /* 只在 Protect_Enable==1 时动作 */
    if (Bind_ProtectEnable()) {
        if (Protect_OcFault) { Protect_ForceStop(); }
    }
}
```

`Protect_UvFault` 的**动作**不在这里做（欠压要切固定 0.85 正弦，属于控制环，放 Task 6）。

- [ ] **Step 4: 实现 `Protect_Init` / `Protect_ForceStop` / `Protect_Clear`**

`Protect_Init` 无条件做（即使 `Protect_Enable == 0`）：`TZCTL` 四个 DCxEVT 全 `TZ_FORCE_LO`、`TZSEL.OSHT = 1`、`TZSEL.DCAEVTn/DCBEVTn = 0`、**不动 `TZEINT`**、`TZCLR.all = 0xFFFF`，EPWM1 和 EPWM2 各做一遍。**完全不碰 `Comp1Regs` / `Comp2Regs` / `TZDCSEL` / `DCACTL` / `DCBCTL`。**

- [ ] **Step 5: 删掉硬件保护残留**

- `main.c`：删 `Epwm1_TripIsr()` 整个函数、删它对应的 `PieVectTable.EPWM1_TZINT` 绑定、删 `PIEIER2` 的 TZ 使能块、删 ISR 里 `#if (Protect_Enable == 1)` 的过压/过流内联判断、删 `Protect_TripFlag` 引用。
- `APP/protect.h`：删 `Protect_TripFlag` / `Protect_TripSource` / `Trip_Amps_Default` / `Use_Comp2_Negative` / `Protect_SetTripLevel` / `Protect_SoftwareCheck` / `Protect_InitOCP`。
- `APP/dcac_cfg.h`：删 `OV_Volts_F` / `OV_Volts`（过压不在本次范围）。

- [ ] **Step 6: 验证编译 + 静态检查**

```powershell
# 编译
$env:PATH = "C:\ti\ccs1040\ccs\utils\bin;" + $env:PATH
& "C:\ti\ccs1040\ccs\utils\bin\gmake.exe" -f makefile -j4 all
# 硬件保护必须一点不剩：
Select-String -Path main.c,APP\protect.c -Pattern "Comp1Regs|Comp2Regs|TZEINT|TZDCSEL|DCACTL|DCBCTL|TripIsr"
```
预期：编译 `EXIT=0`；最后那条搜索**零命中**（`TZCTL` / `TZSEL` / `TZCLR` / `TZFRC` 允许出现）。

- [ ] **Step 7: Commit**

```powershell
git add APP/protect.c APP/protect.h APP/dcac_cfg.h main.c
git commit -m "refactor(protect): replace COMP/DAC/trip hardware protection with software UV+OC"
```

---

### Task 2: 抽出 debug_view.c

**Files:**
- Create: `APP/debug_view.c`、`APP/debug_view.h`
- Modify: `main.c`（删掉搬走的变量和后台计算，改为调用接口）

**Interfaces:**
- Produces:
  ```c
  extern volatile struct { float VoutRms, IoutRms, VoutAmp, IoutAmp, ILRms, ModAmp, Vin, Iin, CalPinIout, CalPinVout, CalPinIL; } Debug_View;
  void Debug_View_Init(void);
  void Debug_View_IsrUpdate(_iq vout, _iq iout, _iq il, _iq iin, _iq mod);  /* 每拍累加，约 20 周期 */
  void Debug_View_ScopeSample(_iq iout, _iq il);                            /* 每拍调用，内部自带抽取 */
  void Debug_View_Background(void);                                        /* 算 RMS / 刷新 Debug_View */
  void Debug_View_SetCalPin(float vo, float io, float il);                  /* Adc_ZeroCal 里调一次 */
  ```

**搬走的东西**（原 `main.c`）：`Debug_View` 结构体、`RmsVoutSq/RmsIoutSq/RmsILSq/AvgVinSum/AvgIinSum`、`FiltVout/FiltIout/FiltIL/FiltMod`、`ModPk`、`ScopeIout/ScopeIL/ScopeIdx/ScopeCnt/Scope_Prescale`、`CalPinVout/CalPinIout/CalPinIL`、`Rms` 的平滑宏 `Rms_SmoothShift`。

**留在 main.c 的东西**：`RmsCnt` / `RmsCommit` 两个计数（ISR 里 2 条指令，Task 5 的 `InvVin` 节拍要挂在上面）、`VoutOffNeg/IoutOffNeg/ILOffNeg`（ISR 换算用）。

- [ ] **Step 1: 建文件骨架 + 接口**

`debug_view.h` 写上面那组声明和 `Debug_View` 结构体。`debug_view.c` 里放搬过来的变量定义（含 `#pragma DATA_SECTION(ScopeIout, "scopeBufs")`）和空函数体。`main.c` 改成调用。编译。

- [ ] **Step 2: 把 ISR 里的累加搬进 `Debug_View_IsrUpdate`**

搬 `RmsVoutSq += ...` 那一段（先右移 8 位再平方）、`AvgVinSum/AvgIinSum`、`ModPk` 取峰值。**示波器抽取单独放 `Debug_View_ScopeSample`**，`Scope_Prescale` 的防 0 逻辑一并搬。

- [ ] **Step 3: 把后台那段搬进 `Debug_View_Background`**

搬"有效值 → 开方 → IIR 平滑 → `Debug_View.*` 赋值"整段，以及 `Debug_View.CalPin*` 的赋值（改成读 `Debug_View_SetCalPin` 存进来的值）。

- [ ] **Step 4: 用 `DEBUG_VIEW_ENABLE` 分层**

`dcac_cfg.h` 常改区加 `#define DEBUG_VIEW_ENABLE 0`。在 `debug_view.c` 里：

```c
/* 下面这几个值 OLED 要用，只要 OLED 开着就必须编译 */
#if !DEBUG_VIEW_ENABLE && (SFRA_ENABLE == 1)
    /* OLED 和调试镜像都关：整块不编译，接口退化成空 */
    #define DEBUG_VIEW_MINIMAL 1
#else
    #define DEBUG_VIEW_MINIMAL 0
#endif
```

`DEBUG_VIEW_MINIMAL` 为 1 时：`Debug_View_IsrUpdate` / `Debug_View_Background` / `Debug_View_ScopeSample` 全是空函数体（编译器会内联掉），`Debug_View` 结构体只保留 `VoutRms` 那 6 个字段。

- [ ] **Step 5: 验证（三种配置都要编译）**

```powershell
# 默认：OLED 版（SFRA_ENABLE=0, DEBUG_VIEW_ENABLE=0）
# 变体一：+ DEBUG_VIEW_ENABLE=1（-dDEBUG_VIEW_ENABLE=1）
# 变体二：SFRA 版（-dSFRA_ENABLE=1 -dDEBUG_VIEW_ENABLE=0）
```
三种都必须 `EXIT=0`。变体二用来证明"OLED 关掉时 `Debug_View` 整块不编译"这条路是通的。

- [ ] **Step 6: 确认 ISR 没引入浮点**

```powershell
& "C:\ti\ccs1040\ccs\tools\compiler\ti-cgt-c2000_20.2.5.LTS\bin\dis2000.exe" main.obj > $env:TEMP\dis.txt
Select-String -Path $env:TEMP\dis.txt -Pattern 'F\$\$'
```
预期：**零命中**。（`Debug_View_Background` 里的浮点在后台，不在 ISR，允许。）

- [ ] **Step 7: Commit**

```powershell
git add APP/debug_view.c APP/debug_view.h main.c APP/dcac_cfg.h
git commit -m "refactor: extract Debug_View / RMS / scope into APP/debug_view.c"
```

---

### Task 3: 抽出 oled_task.c

**Files:**
- Create: `APP/oled_task.c`、`APP/oled_task.h`
- Modify: `main.c`

**Interfaces:**
- Produces:
  ```c
  void Oled_Task_Init(void);              /* 内含 OLED_Init() + OLED_Clear() */
  void Oled_Task_Run(Uint16 isrcnt);      /* 内部用 static 上一次的 isrcnt 做 500ms 节流 */
  ```
- Consumes: `Debug_View`（Task 2）、`Protect_UvFault` / `Protect_OcFault`（Task 1）

- [ ] **Step 1: 搬 `Oled_F` 和 `Oled_Task` 到 `oled_task.c`**

`Oled_Period_Isr` / `OledLastIsr` 一起搬过去，`OledLastIsr` 变成函数内 `static`。

- [ ] **Step 2: 把第 0 行的故障显示改成读新接口**

原来读 `FaultLatch`（`FAULT_HW_OC` / `FAULT_SW_OC` / `FAULT_OVERVOLT`）。改成：

```
S:OK  / S:UV（欠压） / S:OC（过流，锁存）
```

硬件过流那条删掉。

- [ ] **Step 3: 加 `Oled_Task_Init`**

把 `main()` 里 `#if (SFRA_ENABLE == 0) OLED_Init(); OLED_Clear(); #endif` 换成 `Oled_Task_Init()`。

- [ ] **Step 4: 验证**

三种配置都编译（同 Task 2 Step 5）。上板确认 OLED 8 行照常刷新。

- [ ] **Step 5: Commit**

```powershell
git add APP/oled_task.c APP/oled_task.h main.c
git commit -m "refactor: extract OLED display task into APP/oled_task.c"
```

---

### Task 4: 抽出 sfra_task.c

**Files:**
- Create: `APP/sfra_task.c`、`APP/sfra_task.h`
- Modify: `main.c`
- Modify: `APP/sfra_cfg.h`（只保留配置宏和注入/采集宏，函数声明挪到 `sfra_task.h`）

**Interfaces:**
- Produces:
  ```c
  void Sfra_Task_Init(void);          /* 原 SFRA_Init() */
  void Sfra_Task_Background(void);    /* 原主循环里的 SFRA_IQ_BACKGROUND + 自动触发 + SfraDone 判定 */
  ```

- [ ] **Step 1: 搬 `SFRA_ENABLE == 1` 那一整块**

`SFRA1`、`FreqVect`/`Plant_MagVect`/... 五个数组、映射表、`SCIA_GPIO_Init`、`SFRA_Init`、`Sfra_Tick`、`SfraTrig`/`SfraDone`、`SfraAutoFired` 全部搬进 `sfra_task.c`。

- [ ] **Step 2: 主循环那一段改成一次调用**

`main.c` 的 `for(;;)` 里 `#if (SFRA_ENABLE == 1) ... #endif` 整块换成 `Sfra_Task_Background();`。

- [ ] **Step 3: 验证（关键：SFRA 版必须还能编译链接）**

```powershell
# -dSFRA_ENABLE=1 -dSFRA_GUI_ENABLE=0 编译到临时 obj，确认链接不溢出
```
RAM 只有 10K 字，SFRA 版 100 点 × 5 个数组 = 1000 字，改前放得下，改后也不应变化。链接脚本不变。

- [ ] **Step 4: Commit**

```powershell
git add APP/sfra_task.c APP/sfra_task.h APP/sfra_cfg.h main.c
git commit -m "refactor: extract SFRA sweep task into APP/sfra_task.c"
```

---

### Task 5: 电流环前馈

**Files:**
- Modify: `APP/dcac_cfg.h`（`KP_I`/`KI_I` 改 V/A，加 `KP_I3`/`KI_I3`、`Vff_Shift`、`Vin_Ff_Min/Max`、`Vbus_Nominal` 已有）
- Modify: `main.c`（新增 `VoutFF`/`VinFF`/`InvVin`/`V_L` 四个变量、PI 初值条件化、ISR 控制律、后台算 `InvVin`）

**Interfaces:**
- Consumes: `Protect_Update`（Task 1）
- Produces: 文件作用域 `volatile _iq V_L`（电流环 PI 输出 = 电感压降给定，Q24 V）。Step 6 要取它的地址，调试时也直接看它。

- [ ] **Step 1: 改配置区**

```c
#define KP_I            _IQ(9.6f)     /* 电流环 P [V/A] = 0.16 mod/A × 60V */
#define KI_I            0.0f           /* 纯 P */
#define KP_I3           _IQ(0.16f)     /* LOOP_DUAL 仍用调制比量纲 [mod/A] */
#define KI_I3           0.0f
#define Vff_Shift       2U             /* Vout/Vin 一阶 IIR，fc ≈ 795Hz */
#define Vin_Ff_Min_F    55.0f          /* 1/Vin 钳位下限（= 欠压阈值） */
#define Vin_Ff_Max_F    90.0f          /* 钳位上限（Review Focus 2） */
#define Vin_Ff_Min      _IQ(Vin_Ff_Min_F)
#define Vin_Ff_Max      _IQ(Vin_Ff_Max_F)
```

`KI_I_Ts` / `KI_I3_Ts` 两条宏跟着换：

```c
#define KI_I_Ts         _IQ(KI_I  * Ts_S)    /* mode 2，V/A/s */
#define KI_I3_Ts        _IQ(KI_I3 * Ts_S)    /* mode 3，mod/A/s */
```

- [ ] **Step 2: 先用 probe 把几个 Q24 常量钉死**

建临时目录 `_verify\probe.c`：

```c
#include "dcac_cfg.h"
long p_kpi(void)  { return (long)KP_I; }
long p_kpi3(void) { return (long)KP_I3; }
long p_uv(void)   { return (long)UV_Volts; }
long p_oc(void)   { return (long)OC_Amps; }
```

用 `cl2000` 编译后用 `dis2000` 看立即数。预期：

| 常量 | 期望整数 | 真值 | 说明 |
|---|---|---|---|
| `KP_I` | 161 061 273 ~ 161 061 275 | 9.6 | 9.6 × 2²⁴ |
| `KP_I3` | 2 684 353 ~ 2 684 356 | 0.16 | 0.16f 的 float32 表示略小于 0.16 |
| `UV_Volts` | 922 746 880 | 55.0 | 精确 |
| `OC_Amps` | 201 326 592 | 12.0 | 精确 |

（`_IQ(x) = (long)(x × 16777216)`，转换是截断。落在上表区间内即通过。）
验完删掉 `_verify\`。

- [ ] **Step 3: 加滤波变量和后台倒数**

`main.c` 全局：

```c
volatile _iq VoutFF = 0;    /* 输出电压前馈（滤波后） */
volatile _iq VinFF  = _IQ(Vbus_Nominal_F);   /* 母线（滤波后），初值用标称 */
volatile _iq InvVin = _IQ(1.0f / Vbus_Nominal_F);  /* 1/Vin，后台算，开中断前初始化 */
volatile _iq V_L    = 0;    /* 电流环 PI 输出 = 电感压降给定，单位 V（SFRA 要取地址） */
```

后台（`RmsCommit` 节拍里）：

```c
_iq vs = VinFF;
if (vs < Vin_Ff_Min) { vs = Vin_Ff_Min; }
if (vs > Vin_Ff_Max) { vs = Vin_Ff_Max; }
InvVin = _IQdiv(_IQ(1.0f), vs);
```

- [ ] **Step 4: PI 初值改成按模式条件化**

```c
#if (LOOP_MODE == LOOP_CURR_CLOSED)
    /* 前馈结构：PI 输出是电压，限幅每拍在 ISR 里按余量重设，这里只给个初值 */
    PI_I.Kp     = KP_I;                /* V/A */
    PI_I.KiTs   = KI_I_Ts;
    PI_I.OutMin = PI_I.IntMin = _IQ(-(Mod_Max_F * Vbus_Nominal));  /* -51V */
    PI_I.OutMax = PI_I.IntMax = _IQ( (Mod_Max_F * Vbus_Nominal));  /* +51V */
#else
    /* 调制比量纲（LOOP_DUAL 等）：PI 输出直接当调制比 */
    PI_I.Kp     = KP_I3;               /* mod/A */
    PI_I.KiTs   = KI_I3_Ts;            /* dcac_cfg.h: _IQ(KI_I3 * Ts_S) */
    PI_I.OutMin = PI_I.IntMin = -Mod_Max;
    PI_I.OutMax = PI_I.IntMax =  Mod_Max;
#endif
```

`LOOP_DUAL` 那一支保持 `Mod = PI_Calc(&PI_I)`，限幅 `±Mod_Max`。

- [ ] **Step 5: 写 ISR 里的新控制律**

替换 `LOOP_CURR_CLOSED` 分支里 `Mod = PI_Calc(&PI_I);` 之后的部分：

```c
/* 一阶 IIR 滤波 */
VoutFF += (Vout - VoutFF) >> Vff_Shift;
VinFF  += (Vin  - VinFF)  >> Vff_Shift;

/* 动态限幅：只给这一拍真正的电压余量（Review Focus 3） */
{
    _iq head = _IQmpy(Mod_Max, VinFF) - VoutFF;
    if (head < 0) { head = 0; }
    PI_I.OutMax =  head;  PI_I.IntMax =  head;
    PI_I.OutMin = -head;  PI_I.IntMin = -head;
}

V_L = PI_Calc(&PI_I);                    /* Q24 V */
{
    _iq m = _IQmpy(V_L + VoutFF, InvVin);
    Mod = m;
}
```

后面已有的 `±Mod_Max` 限幅保留不动。

- [ ] **Step 6: SFRA 采集点跟着改**

`#if (SFRA_ENABLE == 1)` 里 mode 2 的 `SFRA_COL(&Mod, &IL)` 改成 `SFRA_COL(&V_L, &IL)`（采补偿器输出，不是占空比，理由见 spec 10.5）。`SFRA_ENABLE=0` 时宏是空的，不影响默认编译。因为要取地址，**`V_L` 必须是文件作用域的 `volatile _iq`**，不能是 ISR 局部变量 —— 在 Step 3 那组全局变量里一起定义。

- [ ] **Step 7: 验证 —— 编译 + 反汇编（这次要盯两条）**

```powershell
& "C:\ti\ccs1040\ccs\utils\bin\gmake.exe" -f makefile -j4 all
& "C:\ti\ccs1040\ccs\tools\compiler\ti-cgt-c2000_20.2.5.LTS\bin\dis2000.exe" main.obj > $env:TEMP\dis.txt
Select-String -Path $env:TEMP\dis.txt -Pattern 'F\$\$'      # 预期零命中
Select-String -Path $env:TEMP\dis.txt -Pattern 'IQ.*div|DIV'  # ISR 段里预期零命中
```

第二条要确认命中的都落在 `_Debug_View_Background` / `Adc_ZeroCal` 里，不在 `_ADC_ISR` 段内。

- [ ] **Step 8: 上板验证（台架）**

母线从 55V 慢慢升到 65V，给定 4.16Arms：
- `Mod` 幅值应随母线升高而下降，`Vout` 有效值保持 36Vrms 附近
- `PI_I.Out` 应在 ±1V 量级（电感压降），不再是 ±50V
- 母线 60V 时 `Mod` 峰值 ≈ 0.85（贴着限幅，见 spec 10.4）

- [ ] **Step 9: Commit**

```powershell
git add main.c APP/dcac_cfg.h
git commit -m "feat(current-loop): add Vbus/output-voltage feedforward, PI now outputs volts"
```

---

### Task 6: 欠压兜底

**Files:**
- Modify: `main.c`
- Modify: `APP/dcac_cfg.h`（加 `Uv_Ref_Step`）

**Interfaces:**
- Consumes: `Protect_UvFault`（Task 1）

- [ ] **Step 1: 加欠压兜底斜坡**

```c
volatile _iq UvRamp = 0;    /* 欠压兜底的调制比幅值，0 -> Mod_Max */
```

配置：`#define Uv_Ref_Step _IQ(Mod_Max_F / ((float)Soft_Start_Ms * 0.001f * Fsw_Hz))`

- [ ] **Step 2: 在 `LOOP_CURR_CLOSED` 分支前面插入兜底判断**

```c
if ((Protect_Enable == 1U) && (Protect_UvFault != 0U))
{
    /* 欠压兜底：切固定 0.85 幅值正弦，走自己的软启动（Review Focus 4） */
    if (Run_Enable != 0U) { UvRamp += Uv_Ref_Step; if (UvRamp > Mod_Max) UvRamp = Mod_Max; }
    else                  { UvRamp = 0; }
    Mod = _IQmpy(sin_val, UvRamp);
    PI_I.Integral = 0;          /* 不让闭环历史带进来 */
    IRef = 0;
}
else
{
    UvRamp = 0;                 /* 退出时清零，下次进入重新爬 */
    /* ...原来的电流环 ... */
}
```

- [ ] **Step 3: 故障/清故障路径一并复位 `UvRamp`**

`FaultLatch != FAULT_NONE` 分支和 `FaultClear` 分支各加一句 `UvRamp = 0;`。

- [ ] **Step 4: 验证**

编译三种配置。台架：`Protect_Enable` 临时置 1，把母线从 60V 往下降到 50V，`Mod` 应在 `Vin` 跌破 55V 后 10ms 内变成 0.85 幅值正弦并保持；升回 56V 后自动恢复闭环，且恢复过程积分不冲击。

- [ ] **Step 5: Commit**

```powershell
git add main.c APP/dcac_cfg.h
git commit -m "feat(protect): undervoltage fallback to fixed 0.85-amplitude sine"
```

---

### Task 7: 收尾验证

**Files:**
- Modify: 无（只跑检查）
- Create: 无

- [ ] **Step 1: 三种配置全量编译**

默认（OLED）/ `-dDEBUG_VIEW_ENABLE=1` / `-dSFRA_ENABLE=1`。三次都要 `EXIT=0`，warning 不超过基线。

- [ ] **Step 2: 行数核对**

```powershell
(Get-Content main.c).Count
```
目标 ≤ 700 行（改前 1478）。若超出，把 `Adc_ZeroCal` + `Adc_Config` + 正弦表搬进 `APP/adc_board.c`，并给 `VoutOffNeg/IoutOffNeg/ILOffNeg` 和 `CalPin*` 定义接口。**实际数字写进 commit message。**

- [ ] **Step 3: 反汇编终检**

`_ADC_ISR` 段里：无 `F$$`、无 `IQ.*div`。

- [ ] **Step 4: 台架终检（照 spec 第八节）**

- `Vin > 55V`，4.16Arms 给定：`Mod` 正常跟踪，峰值 ≈ 0.85
- `Vin < 55V`：切 0.85 幅值正弦
- `IsrCyclesMax`（上一轮加的 CpuTimer0 计时）应 < 3000（即 50µs 预算内）

- [ ] **Step 5: Commit + 更新文档**

`docs/superpowers/specs/2026-10-29-...md` 的第八节写回实际行数。

---

## 自审记录

- **spec 覆盖**：前馈→Task 5；保护→Task 1；欠压兜底→Task 6；拆文件→Task 2/3/4；配置区→Task 1 Step 1 + Task 5 Step 1；三种配置编译→Task 7 Step 1。spec 10.5 的两条（SFRA 采集点、Vin 滤波）→Task 4 Step 1 / Task 5 Step 3+6。
- **类型一致性**：`V_L` 在 Task 5 产生、Task 5 Step 6 消费；`Protect_UvFault` 在 Task 1 产生、Task 6 消费；`Debug_View` 在 Task 2 产生、Task 3 消费；`RmsCommit` 全程在 `main.c`。
- **Review Focus 落点**：1→Task 5 Step 3（钳位 + 预初始化）；2→Task 5 Step 1/3（`Vin_Ff_Max`）；3→Task 5 Step 5（`head < 0` 保护）；4→Task 6 Step 1/2（去抖 + 进入清零）；5→Task 2 Step 4（`DEBUG_VIEW_MINIMAL` 分支不假设 `CalPin*` 有效）。
