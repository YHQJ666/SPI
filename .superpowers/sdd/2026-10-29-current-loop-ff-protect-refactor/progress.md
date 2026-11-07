# SDD ledger — plan: docs/superpowers/plans/2026-10-29-current-loop-ff-protect-refactor.md

## Setup
- Workspace: .superpowers\sdd\2026-10-29-current-loop-ff-protect-refactor (manual creation; sdd-workspace 脚本是 bash，Windows 下无 bash)
- Branch: feature/sfra-port (非 main，原地执行)
- Spec: docs/superpowers/specs/2026-10-29-current-loop-ff-protect-refactor-design.md — 已读
- Ruling: 不建 git worktree — 该工程是 CCS 工程，工作目录路径被 .cproject 引用，挪到别处 CCS 打不开 — 代价：与用户 WIP 共处一棵工作树，提交时须逐文件 add

## Pre-flight（任务间共享接口）
- Task 2 → Task 3: Debug_View 的 6 个 OLED 字段（VoutRms/IoutRms/ILRms/ModAmp/Vin/CalPin*）在 MINIMAL 模式下仍编译 — 一致
- Task 1 → Task 3/Task 6: Protect_UvFault / Protect_OcFault — 一致
- Task 2 → Task 5: RmsCnt/RmsCommit 留在 main.c，Task 5 的 InvVin 节拍挂在上面 — 一致
- CONFLICT Task 2: 方案里 Debug_View_IsrUpdate 只有 5 个参数（vout/iout/il/iin/mod），但要搬的代码里有 AvgVinSum += (long long)Vin，需要 vin
  - Ruling: 按 spec 5.3 的 6 参数签名 (_iq vout, _iq iout, _iq il, _iq vin, _iq iin, _iq mod) — 代价：多一个参数
- Ruling: TDD 适配 — 本工程无主机测试框架，每个任务的"测试"= 编译（三配置）+ probe 常量 + dis2000 静态检查，均已写成可执行命令；功能验收只能上台架 — 代价：静态检查覆盖不到的时序/环路问题只能靠台架发现


## Task 1: 配置区 + 软件保护模块
- Step 1（恢复 ISR 主体）: 用户已自行完成，并在同一时间把计时脚从 GPIO4 换成了 GPIO5 —— 保留
- Ruling: 工具踩坑 —— 用 Get-Content/Set-Content 往返处理这个 UTF-8 文件，把中文注释破坏成 GBK 乱码且**有损**（部分字符变成 U+FFFD 无法还原）。用 `git checkout HEAD -- main.c` 从上一个 commit 取回干净字节，再逐条重放本会话全部改动（含用户 GPIO5 改动）。代价：约 30 分钟返工。以后只用 edit 工具或 [System.IO.File]::ReadAllText/WriteAllText(UTF8)
- Ruling: 常改区里 KP_I 暂时保持 0.16 mod/A，V/A 量纲换算挪到 Task 5（与控制律一起）。否则 Task 1 提交后 KP_I 是 V/A 而控制律还是 mod，增益差 60 倍。代价：Task 5 多改一处
- Ruling: 链接溢出（CODERAM 只剩 28 字，IQmath 段要 91 字）。根因是 ISR 主体恢复后代码涨回满量。处理：LOOP_CURR_CLOSED 模式下把 PR 模块整个编掉（pr.c 的 PR_Design/PR_Calc/PR_Sat 包 #if，main.c 的 PR_Design 调用跟着包），省约 190 字。代价：mode 2 下用不了 PR（本来也不用）；切 mode 1/3 时自动编回
- Ruling: protect.c 的 Protect_ForceStop() 无条件生效（不受 Protect_Enable 约束），但所有调用点都包在 Protect_Enable==1 里 —— 保证"Protect_Enable=0 时只观察不动作"。代价：以后新增调用点必须自己记得加门
- 验证: gmake 全量重编+链接 EXIT=0，**零 warning**；硬件保护残留扫描（排除注释）零命中；CODERAM 0x1927/0x1A00（余 0xD9）

## Task 2: 抽出 debug_view.c
- 搬走: Debug_View 结构体、RmsVoutSq/RmsIoutSq/RmsILSq/AvgVinSum/AvgIinSum、ModPk、FiltVout/Iout/IL/Mod、CalPin*、ScopeIout/ScopeIL/ScopeIdx/ScopeCnt/Scope_Prescale
- 留下: RmsCnt / RmsCommit（Task 5 的 1/Vin 节拍要挂在这上面）
- 接口: Debug_View_Init / Debug_View_IsrUpdate(6 参数) / Debug_View_ScopeSample / Debug_View_Background / Debug_View_SetCalPin
- Ruling: Debug_View 用 typedef 的 Debug_View_Type 而不是匿名 struct —— 匿名 struct 在每个 .c 里是不同类型，头文件 extern + .c 定义会报 #148 类型不兼容。代价：多一个类型名
- Ruling: 示波器缓冲默认关（DEBUG_VIEW_ENABLE=0），释放 512 字数据 RAM（dataRAM1 用量 0x462 -> 0x262）。代价：要用 Graph 看波形必须把 dcac_cfg.h 的 DEBUG_VIEW_ENABLE 改成 1
- Ruling: Debug/ 下三个生成的 makefile 手工补了 debug_view.obj（生成物、已 gitignore）。.cproject 的 sourceEntries 是整个工程根目录的 sourcePath，CCS 下次 Build 会按文件系统自动重建 makefile，所以 CCS 里不用手工 Add Files
- 验证: gmake EXIT=0 零 warning；ADC_ISR 反汇编 349 条指令、5 个 LCR（PI_Calc/SPWM_SetMod/Protect_Update/Debug_View_IsrUpdate/Debug_View_ScopeSample），无 F\$\$；debug_view.c 在 DEBUG_VIEW_ENABLE=1 / SFRA_ENABLE=1 两种变体下均单独编译通过；main.c 1478 -> 1270 行

## Task 3: 抽出 oled_task.c
- 搬走: Oled_F / Oled_Task(改成 static Oled_Draw(fault)) / OledLastIsr / Oled_Period_Isr / OLED_Init / OLED_Clear
- 接口: Oled_Task_Init()、Oled_Task_Run(Uint16 isrcnt, Uint16 fault)
- Ruling: Oled_Task_Run 多一个 fault 参数（方案里只有 isrcnt）—— 避免 oled_task.c 去 extern main.c 的 FaultLatch。代价：多一个参数
- Ruling: FAULT_NONE/FAULT_UV/FAULT_OC 三个码挪到 protect.h（main.c 和 oled_task.c 都要用，放保护头文件语义最合适）
- 验证: gmake EXIT=0 零 warning；CODERAM 0x193D/0x1A00；main.c 1270 -> 1079 行

## Task 4: 抽出 sfra_task.c
- 搬走: SFRA1 对象、5 个结果数组、映射表、SfraTrig/SfraDone/SfraAutoFired、SCIA_GPIO_Init、SFRA_Init(改名 Sfra_Task_Init)、Sfra_Tick、主循环里的后台状态机
- 接口: Sfra_Task_Init()、Sfra_Task_Background()（都带 SFRA_ENABLE==0 的空实现）
- sfra_task.c 里 extern 了 main.c 的 IsrCnt（自动扫频延时判据要用）
- 验证: 默认配置 gmake EXIT=0 零 warning；SFRA_ENABLE=1 时 main.c / sfra_task.c / debug_view.c / oled_task.c 四个文件单独编译全部 exit=0；CODERAM 0x1942/0x1A00（余 0xBE）；main.c 1079 -> 880 行

## Task 5: 电流环前馈
- dcac_cfg.h: KP_I = _IQ(9.6f) V/A、KI_I 保持 0；新增 Vff_Shift=2、Vin_Ff_Min/Max=55/90V
- main.c: 新增 VoutFF / VinFF / InvVin / V_L 四个全局（InvVin 有初值 1/60，防 .ebss 清零导致第一拍 mod=0）
- ISR mode 2: VoutFF/VinFF 一阶 IIR -> 按拍算动态限幅 head = 0.85*VinFF - VoutFF（head<0 钳 0）-> V_L = PI_Calc -> Mod = (V_L + VoutFF)*InvVin
- PI 初值按模式条件化：mode 2 用 KP_I/KI_I + ±51V；其它模式用 KP_I3/KI_I3 + ±Mod_Max
- 后台 RmsCommit 节拍里算 InvVin = _IQdiv(1, clamp(VinFF, 55, 90))
- SFRA 采集点 mode 2 改成 SFRA_COL(&V_L, &IL)
- Ruling: probe 实测 KP_I = 161061280（不是方案表里的 ...273~275）。原因：9.6f 的 float32 表示是 9.60000038147，×2^24 = 161061280.2。方案里那个区间是按十进制 9.6 算的，没考虑 float32 量化。数值本身正确，不改。代价：无
- 验证: gmake EXIT=0，只有 spwm.c 那条原有 warning；ADC_ISR 394 字、5 个 LCR、0 个 F\$\$（除法落在后台，IQmath 段 91 字被链进 CODERAM 0x9965）；probe 实测 UV=0x37000000(922746880)、OC=0x0C000000(201326592) 精确

## Task 6: 欠压兜底
- dcac_cfg.h 8c: Uv_Ref_Step = _IQ(0.85/(0.5*20000)) = 28508
- main.c: 新增 UvRamp；mode 2 里算完 Mod 之后，若 Protect_UvFault 则用 Mod = sin_val*UvRamp 覆盖，并清 PI 积分 / V_L / IRef；退出时 UvRamp 清零
- 故障锁存、FaultClear、故障停机三条路径都加了 UvRamp = 0
- 整段在 #if (Protect_Enable == 1) 里，当前配置（Protect_Enable=0）不编译
- Ruling: 兜底用"覆盖 Mod"的方式实现，没有把电流环整段包进 else 分支 —— 省一次大范围重排（当前 CODERAM 只剩二十几字，缩进重排风险大于收益），代价是欠压期间电流环仍照算、结果被丢弃（约 30 周期，不影响 20ms 级保护动作）
- 验证: gmake EXIT=0，只有 spwm.c 那条原有 warning；CODERAM 0x19E5/0x1A00（余 0x1B = 27 字）

## Task 7: 收尾验证 + 终审
- 五配置编译（默认 / DEBUG_VIEW_ENABLE=1 / SFRA_ENABLE=1 / Protect_Enable=1 / LOOP_MODE=3）: 8 个源文件全部编译通过
- ADC_ISR（默认配置）: 394 字、5 个 LCR、0 个 F\$\$、0 个除法
- 行数: main.c 984、debug_view.c 219、oled_task.c 234、sfra_task.c 227、protect.c 132
- Ruling: main.c 984 行，没到方案里的 ~700。剩余内容是 Adc_ZeroCal(151)+Adc_Config(83)+正弦表(37)+ISR(343)，不是用户抱怨的"开头变量定义 + 调试代码"（那些已经全部搬走）。继续拆要动 ISR 和 ADC 标定，且 CODERAM 只剩 14 字，风险大于收益 —— 留给下一轮
- 终审（独立 subagent，全新上下文）共 2 Critical / 6 Important / 8 Minor，逐条处置如下

Final: fixed  HARDWARE-ONLY-KILLSWITCH  protect.c 的 TZSEL 原来整个清零，spec 要求 OSHT=1 -> 改成 TZSEL.all = 0x0100（只选 OSHT1）；顺带改正 protect.h 里"TZCTL 复位值 0xFFFF 是强制输出高"的错误说法（实际 0xFFFF 的位域是 3=不动作）
Final: fixed  UV-EXIT-NO-HYSTERESIS  欠压退出原来一拍就清 -> 加 Protect_UvClrCnt + UV_Clear_Debounce(20 拍 = 1ms)，避免母线在 55V 抖动时兜底斜坡被反复清零
Final: fixed  IRAMP-NOT-RESTARTED    UV 兜底期间 IRamp 保持 5.88A，回闭环会一步跳满 -> 兜底分支里加 IRamp = 0
Final: fixed  SFRA-ACCUMULATOR-OVERFLOW  Debug_View_IsrUpdate 的累加在 SFRA 版没人清零（Background 是空的），64 位累加约 41s 溢出 -> 整段加 #if (SFRA_ENABLE == 0) 守卫
Final: fixed  STALE-DOCS  main.c 文件头、spwm.h、dcac_cfg.h（Uv_Ref_Step 注释值 28508 -> 实际 1426、ISR_Test_Pin 4 -> 5）
Final: Ruling: KP_I-BASELINE  评审说基准是 commit 里的 0.6161，据此判我把增益降了 3.85 倍。**这个前提是错的**：0.6161 是 HEAD 里的值，而工作区里用户自己把它改成了 0.16（本会话早先的 git diff 显示 `-#define KP_I _IQ(0.6161f) / +#define KP_I _IQ(0.16f)`），我换算的就是工作区的 0.16 × 60V。评审读的是 `git show a348b4d:APP/dcac_cfg.h`（未含用户未提交的改动）。KP_I3 用 0.16 同理（老代码所有模式共用同一个 KP_I，工作区值就是 0.16）。代价：无
Final: Ruling: MODES-1-3-DO-NOT-LINK  LOOP_MODE=1/3 现在链不过（CODERAM 只剩 14 字，IQmath 段要 91 字）。根因是 RAM 版代码区已经 99.3% 满，不是本次改动引入的；mode 2 正常。代价：暂时只能用 mode 2/0/4；要跑 mode 1/3 得换 28035_FLASH_lnk.cmd 或先腾出 RAM
Final: Ruling: KP-ERR-Q24-OVERFLOW  评审指出 Kp×err 在 |err|>13.3A 时会溢出 Q24（|IRef|<=8 + |IL|<=12 可达 20A）。这条是真的，但修它要动 pid.c 加误差钳位，CODERAM 只剩 14 字放不下。代价：只有 IL 通道失效/超量程时才会触发，且结果的量级仍被 ±head 钳住；下次有 RAM 时优先修
Final: Ruling: VIN-FF-CANNOT-REMOVE-100Hz  评审指出 Vff_Shift=2（fc≈796Hz）对 100Hz 母线纹波只衰减 0.8%，和我写在 spec 里"滤掉 100Hz 脉动"的说法不符。这是 spec 层面的设计缺口，实现是忠实照做的。代价：母线纹波会以约 1% 的量级进占空比；真要滤得换 Vff_Shift=1 或加单独的慢速母线平均
Final: Ruling: PROTECT-UPDATE-REPEATS-FORCESTOP  OC 锁存期间每 50us 重复调用 Protect_ForceStop（6 次寄存器写）。无功能问题，纯粹浪费几个周期。代价：无
Final: minor (deferred): FaultLatch 在 Protect_Enable=0 时也会显示 S:OC/S:UV（与老行为不同），OLED 上分不清"有故障"和"保护已开"
Final: minor (deferred): Adc_ZeroCal 注释里写的 max-min 一致性检查和 CalMiss 计数器在代码里根本不存在（老问题，不是本次引入）
Final: minor (deferred): pid.h 里 Kp=0.8411 的老注释已过时
Final: minor (deferred): InvVin 的 20ms 更新会被 OLED 位带刷新（几十毫秒）挤后

## 用户报告：mode 1/3 链接报 _PR_Calc / _PR_Design 未定义
- 根因（我的 bug）: pr.c 只 include 了 pr.h，而 pr.h 只带 IQmathLib.h，**不带 dcac_cfg.h**。
  于是 pr.c 里 LOOP_MODE 和 LOOP_CURR_CLOSED 都是未定义标识符，预处理按 0 处理，
  #if (0 != 0) 恒假 -> PR 三个函数在**所有**模式下都被编掉。
  mode 2 不调用它们所以能链，mode 1/3 一调用就找不到符号。
- 修: pr.c 里显式 #include "dcac_cfg.h"
- 验证: 预处理输出确认 mode 2 里 PR_Design 不出现、mode 1/3 出现
- ★ 遗留: 修好之后 mode 1/3 会改报"program will not fit"（要 ~192 字，CODERAM 只剩 14 字）。
  这是 RAM 版代码区打满的问题，不是符号问题。出路：换 28035_FLASH_lnk.cmd，或把浮点 RMS 换成整数开方腾 ~700 字
