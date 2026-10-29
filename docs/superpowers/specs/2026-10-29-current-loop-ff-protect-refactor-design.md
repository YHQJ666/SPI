# 电流环前馈 + 保护简化 + main.c 拆分 设计

日期：2026-10-29
工程：`SPI_LC_iq`（F28035 单相离网逆变，20kHz）
状态：已过一轮 builder check，评审提问已全部消化进本文（见第九节）

---

## 一、为什么做

三件事凑在一起，都在同一条代码路径上：

1. **电流环没有前馈**，PI 输出直接当调制比。母线 60V 时增益"看起来对"，
   母线一变增益跟着变，而且要把 50.9V 的输出电压全靠误差积累出来。
   参考 TI 逆变器设计（版本 2，闭环电流环），式 (12)：

   ```
   invDutyPU = [(invLiRefInst − invLiInst) × Gi + invVoltInst] / invVbusInst
   ```

   即 `mod = (PI 输出 + 输出电压) / 母线电压`。两个好处：
   母线电压被除掉，**环路增益与母线无关**；输出电压被前馈掉，
   **PI 只负责电感压降这一项**。

2. **硬件保护用不上**。现在是 COMP1/COMP2 + 内部 DAC + Digital Compare + TZ 中断，
   需要比较器的模拟通路，板上没有。

3. **main.c 太臃肿**：开头上百行变量定义（采样、校零、RMS、OLED、示波器、SFRA），
   实际天天要改的只有 `LOOP_MODE` / `SFRA_ENABLE` / Kp / Ki。

### 一个必须知道的事实（前馈的真正价值）

输出 36Vrms = 50.9V 峰值，母线 60V，`Mod_Max = 0.85` → 桥臂最大 51V。
**正弦峰值处的电压余量只有 51 − 50.9 ≈ 0.1V。**

- 没有前馈时：PI 要自己凑出 50.9V，`Kp = 9.6 V/A` 意味着误差 5.3A 就把输出顶满，
  这个电流环根本没法工作。
- 有了前馈：PI 只需要产出 `L·di/dt`（约 0.55V），余量绰绰有余。

所以前馈不是"锦上添花"，是这个工况能跑起来的前提。

---

## 二、目标

- 电流环加母线电压 + 输出电压前馈，结构照式 (12)。
- 删掉整套硬件保护（COMP/DAC/Digital Compare/TZ 中断）。
- 只保留两条**软件判断**，**当前置 0，只判断不动作**：
  - 欠压 `Vin < 55V`
  - 过流 `|IL| > 12A`（瞬时峰值）
- 欠压动作：切到 50Hz 正弦、幅值 0.85 的开环兜底。
- `main.c` 拆开，调试镜像 / OLED / SFRA 各自成文件。
- 可调参数集中到 `dcac_cfg.h` 最上面一段。

## 三、非目标

- 不改 `LOOP_SINE_OPEN`(0) / `LOOP_VOLT_CLOSED`(1) / `LOOP_DUAL`(3) / `LOOP_FIXED_DUTY`(4)
  的控制行为。欠压兜底只在 `LOOP_CURR_CLOSED`(2) 生效。
- 不动 SFRA 库配置和扫频流程。
- 不重新整定 Kp/Ki，只做量纲换算（60V 下环路增益不变）。
- 不改硬件采样标定 `ADC_*_K` / `Vzero`。

## 四、约束

- 20kHz 中断里**不能有浮点**，也避免 64/32 除法（约 40 周期）。
- `GLOBAL_Q = 24`，动态范围 ±128。前馈后 PI 输出最大约 ±51V，`1/Vin ≈ 0.018`，都装得下。
- RAM 10K 字；主要是搬家，净增很小。
- 链接脚本不变（`28035_RAM_lnk.cmd`）。

---

## 五、怎么做

### 5.1 电流环前馈（`LOOP_CURR_CLOSED`）

ISR 里：

```c
PI_I.Ref = IRef;                       // A
PI_I.Fdb = IL;                         // A

// 动态限幅：PI 只能用到"这一拍真正的电压余量"
head = _IQmpy(Mod_Max, VinFF) - VoutFF;
if (head < 0) { head = 0; }            // ★ 母线跌落时 head 会变负，不挡会让 OutMax<OutMin
PI_I.OutMax  =  head;   PI_I.IntMax  =  head;
PI_I.OutMin  = -head;   PI_I.IntMin  = -head;

_V   = PI_Calc(&PI_I);                 // 单位 V：电感压降给定 L*di/dt
Mod  = _IQmpy(_V + VoutFF, InvVin);    // (V_电感 + V_输出) / Vin
限幅 ±Mod_Max
```

**动态限幅的理由**：改成电压量纲以后，PI 面对的饱和边界不再是 `±Mod_Max`，
而是 `±(Mod_Max·Vin − Vout)`。正弦峰值处这个值只有 0.1V。如果用固定的 ±51V，
`PI_Calc` 的抗饱和判据永远不成立（它只在自身 OutMax/OutMin 处停止积分），
积分会一路涨到 51V 而占空比早就限在 0.85 —— 典型积分饱和。
按拍改限幅就解决了，**`pid.c` 一行都不用动**。

**(a) PI 量纲：调制比 → 电压**

| 参数 | 现在 | 改后 | 换算 |
|---|---|---|---|
| `KP_I` | 0.16 (mod/A)，纯 P | **9.6** (V/A) | × `Vbus_Nominal`(60) |
| `KI_I` | 0.0（纯 P） | **0.0**（保持纯 P） | ×60 仍是 0 |
| `PI_I.Out/IntMin/Max` | ±0.85 | 每拍动态算 | 见上 |

当前值 `KP_I = 0.16`、`KI_I = 0` 是纯 P。保持纯 P 是合理的：前馈之后
被控对象是 `1/(sL)`，纯 P 的环路增益在 50Hz 处 `Kp/(ωL) = 9.6/0.094 ≈ 102`，
静差约 1%，够用。**不需要加积分。**

**(b) `VoutFF`：输出电压一阶低通**

ADC 的 `Vout` 带开关纹波和死区尖峰，直接前馈等于把噪声灌进占空比。

```c
VoutFF += (Vout - VoutFF) >> Vff_Shift;    // Vff_Shift 默认 2
```

`Vff_Shift=2` → 转折频率 ≈ 20k/(2π·4) ≈ 795Hz，对 50Hz 相位滞后 ≈ 3.6°。

**(c) `VinFF` + `InvVin`：母线也要滤，倒数在后台算**

母线是直流，但单相逆变器的输入电流是脉动的，母线本身带 100Hz 纹波。
`1/Vin` 只要带 1V 误差，占空比就有 1.7% 的幅值误差，所以母线同样要滤：

```c
VinFF += (Vin - VinFF) >> Vff_Shift;       // ISR 里，和 VoutFF 同一个 shift
```

倒数**不在 ISR 里算**（母线变化是毫秒级）：

- 后台每 20ms 一次：`InvVin = _IQdiv(_IQ(1.0f), VinSafe)`
- `VinSafe = VinFF` 钳到 `[Vin_Ff_Min, Vin_Ff_Max] = [55V, 90V]`
  （闭环只在 `Vin ≥ 55V` 时运行，这层钳位防上电瞬间和下溢）
- ISR 里一次 `_IQmpy` 搞定

**这个 20ms 节拍由 `main.c` 自己拥有**，不放在调试模块里 ——
`DEBUG_VIEW_ENABLE=0` 时它也必须存在（详见 5.3）。

`InvVin` 必须在开中断之前初始化成 `1/Vbus_Nominal`，否则 BSS 里的 0
会让第一拍 `Mod = 0`。

**ISR 新增运算**：2 次 `_IQmpy`、1 次加法、2 次移位减法、4 次限幅赋值。
相比 `PI_Calc` 可忽略。

### 5.2 保护：删硬件，留软件两条

**删掉**：

- `Comp1Regs` / `Comp2Regs` 全部配置、`Protect_AmpsToDacCode()`
- `Protect_ConfigEpwmTrip()`（`TZDCSEL` / `DCACTL` / `DCBCTL` / `TZSEL` 里的 DCxEVT）
- `EPwm1.TZEINT`、`Epwm1_TripIsr()`、`Protect_TripFlag` / `Protect_TripSource`
- `Protect_SetTripLevel()`（没有 DAC 了）

**保留并改写**：

```c
void    Protect_Init(void);               // 清状态 + 配 TZ 软件封波
void    Protect_Update(_iq vin, _iq il);  // ISR 每拍调用
void    Protect_Clear(void);              // 清锁存
void    Protect_ForceStop(void);          // 软件封波 TZFRC.OST
extern volatile Uint16 Protect_UvFault;
extern volatile Uint16 Protect_OcFault;
```

| 项 | 条件 | 去抖 | 动作（仅 `Protect_Enable==1`） |
|---|---|---|---|
| 欠压 UV | `Vin < UV_Volts(55V)` | `UV_Debounce` = 200 拍(10ms) | **不锁存**，切固定 0.85 正弦，电压回来自动恢复 |
| 过流 OC | `\|IL\| > OC_Amps(12A)` | `OC_Debounce` = 3 拍 | **锁存**，软件封波，要 `FaultClear` 才恢复 |

**两条我的决定（请重点看）**：

1. **TZ 不整套删，留一个"软件封波"开关。** `TZCTL` 配成 force-low、`TZSEL.OSHT=1`、
   **不开** `TZEINT`。理由是 `AQCSFRC` 封不住 EPWMxB（死区模块会取反，
   `spwm.h` 末尾记过这个坑），而 TZ 不需要任何外部比较器，和"删硬件保护"不冲突。
   **这个配置无条件做**，即使 `Protect_Enable=0` —— 没有 DC 事件源时它只能靠
   `TZFRC` 软件触发，等价于惰性。
2. **欠压的"固定 0.85"= 正弦幅值 0.85**，不是调制比恒等于 0.85（那是直流，
   会把电感单向充起来）。

**欠压兜底的软启动**：幅值走自己的斜坡 `UvRamp`，`Run_Enable` 门控，
`Soft_Start_Ms`(500ms) 内从 0 爬到 `Mod_Max`，**每次进入 UV 状态时清零**。
这样上电（`Vin=0` 本身就是 UV）和电压跌落都不会把 0.85 全幅一步打进 LC。

```
Mod = _IQmpy(sin_val, UvRamp);
```

### 5.3 拆分 main.c

```
main.c              初始化 + ADC_ISR（控制环）+ 后台调度
APP/debug_view.c    Debug_View 浮点镜像、RMS/平均累加、示波器缓冲
APP/oled_task.c     OLED 显示任务
APP/sfra_task.c     SFRA 初始化、库对象、结果数组、后台状态机
APP/protect.c       软件保护（重写）
APP/dcac_cfg.h      配置
```

**接口**：

```c
// debug_view.h
void Debug_View_Init(void);
void Debug_View_IsrUpdate(_iq vout, _iq iout, _iq il, _iq iin, _iq mod);  // ISR 钩子
void Debug_View_Background(void);      // 算 RMS / 刷新
void Debug_View_ScopeSample(_iq iout, _iq il);

// oled_task.h
void Oled_Task_Init(void);
void Oled_Task_Run(Uint16 isrcnt);

// sfra_task.h
void Sfra_Task_Init(void);
void Sfra_Task_Background(void);
```

**`DEBUG_VIEW_ENABLE` 的边界（评审提的第 3 点，这里定死）**：

`DEBUG_View` 里的 `VoutRms / IoutRms / ILRms / ModAmp / Vin / CalPin*`
是 **OLED 唯一的数据来源**，所以它们不能受 `DEBUG_VIEW_ENABLE` 控制。
分成两层：

| 宏 | 控制什么 |
|---|---|
| `OLED_ENABLE`（= `SFRA_ENABLE==0`） | RMS 计算 + `Debug_View` 里 OLED 用到的那 6 个值 |
| `DEBUG_VIEW_ENABLE`（默认 0） | 额外字段（`VoutAmp/IoutAmp/Iin`）、示波器缓冲 `ScopeIout/ScopeIL`、整个 `Debug_View_IsrUpdate` 的扩展部分 |

`DEBUG_VIEW_ENABLE=0` 且 OLED 也关（SFRA 版）时，`Debug_View` 整块不编译，
ISR 里连函数调用都没有。

**ISR 里的累加**：RMS 平方和必须每点累加。保留成 `Debug_View_IsrUpdate()`
一次函数调用（5 个参数，约 20 周期）；宏关掉时展开成空。
`RmsCnt/RmsCommit` 两个计数留在 `main.c`（ISR 里 2 条指令），
因为 5.1(c) 的 `InvVin` 后台节拍要挂在这上面。

### 5.4 配置集中到 dcac_cfg.h 最上面

```c
// ======== 常改区 ========
#define LOOP_MODE           2      // 0 正弦开环 / 1 电压单环 / 2 电流单环 / 3 双环 / 4 固定占空比
#define SFRA_ENABLE         0      // 1 = 扫频版（无 OLED）；0 = OLED 版
#define DEBUG_VIEW_ENABLE   0      // 1 = 额外镜像字段 + 示波器缓冲

// 电流环（LOOP_CURR_CLOSED）Kp/Ki，单位 V/A —— 前馈后 PI 输出是电压
#define KP_I                _IQ(9.6f)     // = 0.16 mod/A × 60V
#define KI_I                0.0f          // 纯 P

// 双环（LOOP_DUAL）仍用调制比量纲，单独一套，互不影响
#define KP_I3               _IQ(0.16f)    // [mod/A]
#define KI_I3               0.0f

#define UV_Volts_F          55.0f         // 输入欠压 V
#define OC_Amps_F           12.0f         // 电感过流 A（峰值）
#define Protect_Enable      0             // 0 = 只判断不动作
// ========================
```

`LOOP_DUAL` 的 PI 初值改成条件编译：mode 2 用 `KP_I/KI_I` + 动态电压限幅；
mode 3 用 `KP_I3/KI_I3` + `±Mod_Max`。否则 mode 3 会被静默放大 60 倍。

---

## 六、留给实现的决定

- `Vff_Shift` 默认 2；`UV_Debounce` 200 拍；`OC_Debounce` 3 拍
- 文件切分位置、函数命名、局部变量摆放
- UV 兜底退出、回闭环时：清 PI 积分 + `IRamp` 重新爬
- `Adc_ZeroCal` / `Adc_Config` / 正弦表是否也搬出去（见第八节）

## 七、请你看这几条（都是我的决定，不是你说过的）

1. 欠压兜底 = 正弦幅值 0.85，**且走自己的 500ms 软启动**、每次进 UV 清零。
2. 过流动作你没定，我先写成"锁存 + 软件封波"，`Protect_Enable` 还是 0，等你以后开。
3. **TZ 保留成纯软件封波开关**（不要外部比较器），其余硬件保护全删。
4. `KP_I` 从 0.16 变成 9.6（量纲变 V/A，60V 下行为不变）；`LOOP_DUAL` 单独给 `KP_I3`。
5. PI 限幅改成**每拍按电压余量动态算**（不改 `pid.c`）。

## 八、验收

- `gmake` 全量重编 + 链接通过，无新增 warning。
- 反汇编确认 20kHz 中断里没有 `F$$` 浮点调用、没有除法调用。
- 默认配置下 `main.c` **目标** 700 行以内（原 1478 行）。
  评审核算：搬走三块后约 940 行，再搬 `Adc_ZeroCal`(~144) + `Adc_Config`(~83) + 正弦表(~35)
  可到 670 左右。**这条是目标不是硬指标**，实际数字写进实施方案。
- 上电 `Vin > 55V`：4.16Arms 给定下 `Mod` 正常跟踪，峰值处 `Mod ≈ 0.85`。
- `Vin` 拉到 55V 以下：`Mod` 变成 0.85 幅值正弦；拉回去自动恢复闭环。

## 九、评审提问的消化记录

| # | 提问 | 处理 |
|---|---|---|
| 1 | 固定 ±51V 限幅会不会积分饱和 | **采纳**，改每拍动态限幅（5.1） |
| 2 | Kp/Ki 基准值、要不要加积分 | 以当前文件 `0.16 / 0` 为基准 ×60；**保持纯 P**（5.1a） |
| 3 | `DEBUG_VIEW_ENABLE=0` 会让 OLED 没数据 | **采纳**，拆成 `OLED_ENABLE` / `DEBUG_VIEW_ENABLE` 两层（5.3） |
| 4 | 共享 `PI_I` 会让 `LOOP_DUAL` 增益 ×60 | **采纳**，mode 3 单独 `KP_I3/KI_I3`（5.4） |
| 5 | `InvVin` 用原始抽样还是平均值、要不要滤 | **采纳**，加 `VinFF` 滤波，倒数用滤波值（5.1c） |
| 6 | 欠压兜底要不要跟软启动、是否影响 mode 1/3 | **采纳**，`UvRamp` + 只在 mode 2（5.2） |
| 7 | 400 行是不是硬指标 | 改成目标值，实际数字进实施方案（八） |

## 十、和工作区里的 TI 例程对照

参照：`C:\Users\6\workspace_guide\SPI\voltagesourceinvlcfltr`
（TI 单相逆变 + LCL 滤波参考设计，Build Level 2 = 闭环电流环）

### 10.1 前馈那一行，一字不差

`voltagesourceinvlcfltr.c` 的 `inverterISR()`，第 674~676 行（Build 2）和
第 738~740 行（Build 3）：

```c
invDutyPU = (cntl3p3z_InvI_vars.Out + invVoInst) / invVbusInst;
invDutyPU = (invDutyPU >  0.98f) ?  0.98f : invDutyPU;
invDutyPU = (invDutyPU < -0.98f) ? -0.98f : invDutyPU;
```

和本设计的 `Mod = (_V + VoutFF) × InvVin` 是同一个式子，只有三处差别：

| | TI 例程 | 本工程 |
|---|---|---|
| 基准 | 标幺（PU），基准被 sensing scaling 吸收 | 实际量纲（V、A） |
| 补偿器输出 | 电压 PU | 电压 V |
| 母线处理 | 直接除 `invVbusInst`（PU） | 后台算 `1/Vin`，ISR 里乘 |
| 占空比上限 | ±0.98 | **±0.85**（本工程 `Mod_Max`，对应 36Vrms@60V） |

### 10.2 标幺化把"量纲问题"吸收掉了 —— 为什么本工程必须显式处理

`hv1phdcac_board.h` 的 `readCurrVolADCSignals()`（第 365~378 行）：

```c
invVoInstADC = ((float32)(VAC_FB) * ADC_PU_SCALE_FACTOR - 0.5) * 2.0;
invIiInstADC = ((float32)(...IINV_FB1..4 之和) * ADC_PU_SCALE_FACTOR * 0.25 - invIiOffset) * 2.0;
invVbusInst  = ((float32)(VBUS_FB)) * ADC_PU_SCALE_FACTOR + invVbusOffset;
```

三路全在 `readCurrVolADCSignals()` 里就 scale 成 PU，所以式 (12) 天生量纲自洽，
`Gi` 的系数（图 27 的 `Kp = 0.503949`）是在 PU 域设计的。
本工程不做 PU，所以必须显式把 `KP_I` 从 mod/A 换成 V/A（×60），
并把 PI 限幅从"调制比"换成"电压"——就是 5.1 那两条。

### 10.3 补偿器的饱和 —— 和本设计的动态限幅是同一套结构

`CNTL_3P3Z_F.h` 的 `CNTL_3P3Z_F_COEFFS` 里有三个限幅字段：

```c
float32 Max;   // 输出上限
float32 IMin;  // 积分限
float32 Min;   // 输出下限
```

也就是 TI 的补偿器同样有**输出限幅 + 积分限幅**，和本工程 `PI_Calc` 的
`OutMax/OutMin/IntMax/IntMin` 是一回事（连抗饱和判据都一样：
输出限幅时才停止积分）。

差别只在限幅值的来源：TI 是补偿器设计器算出来的**静态值**；
本工程按当前母线的**电压余量动态算**。本工程必须动态算的原因见 10.4。

### 10.4 本工程的电压余量比 TI 例程小得多 —— 这是关键差异

TI 在 `voltagesourceinvlcfltr.c` 第 509 行明确写了母线选型假设：

```c
// assuming a ~10% margin on the DC Bus vs Typical AC Output Max
```

也就是 TI 的母线按"输出峰值 × 1.1"选，占空比上限又给到 0.98，
所以它的补偿器永远有余量，静态限幅够用。

本工程是 36Vrms（峰值 50.91V）/ 母线 60V / `Mod_Max = 0.85`：

```
桥臂最大输出 = 0.85 × 60 = 51.0V
正弦峰值处余量 = 51.0 − 50.91 = 0.1V
```

**峰值处只有 0.1V 余量**，静态限幅在这里必然积分饱和，所以必须按拍算余量。
好消息是余量和需求是互补的：正弦峰值处 `di/dt = 0`，PI 本来就只需要输出 0V；
`di/dt` 最大（过零）处 `Vout ≈ 0`，余量正好 51V。两者不冲突。

> **由此有一个必须提前知道的物理限制**：工作在 36Vrms/60V 时，
> 占空比在整个正弦周期里几乎贴着 0.85 上限跑（峰值处只有 0.1V 余量）。
> 所以正弦峰值附近出现削顶/波形失真是这条规格本身决定的，不是环路没调好。
> 想彻底避免只有两条路：把输出目标降到 ~32Vrms 以下，或把母线提到 65V 以上。

### 10.5 另外两个从例程带过来的实现细节

1. **SFRA 采集点要跟着改**。例程第 763 行是
   `SFRA_F_COLLECT(&cntl3p3z_InvI_vars.Out, &invIiInst)` —— 采的是**补偿器输出**
   （电压），不是占空比。因为前馈/除母线之后，占空比到电流不再是那个受控对象。
   本工程 mode 2 现在是 `SFRA_COL(&Mod, &IL)`，要改成 `SFRA_COL(&_V, &IL)`。
   （`SFRA_ENABLE=0` 时这个宏是空的，不影响当前编译。）

2. **例程的母线不做滤波、也不做除零保护**（见 10.2 的采样函数）。
   本工程照 5.1(c) 加 `VinFF` 滤波和 `VinSafe` 钳位 —— 因为余量太小，
   母线上一伏的纹波就是 1.7% 的占空比误差。
